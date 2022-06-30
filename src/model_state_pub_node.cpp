/**
 * @ Author: Kenneth Simon
 * @ Email: smkk00715@gmail.com
 * @ Create Time: 2022-05-20 22:58:00
 * @ Modified time: 2022-05-31 00:11:42
 * @ Description: This node is used to publish the ground truth pose and twist of the selected models in Gazebo.
 * The reason of not using the topic /gazebo/model_states is that the search time for a model's states from the topic's
 * massage is too long and it will also cost too much data bandwidth when there are a large number of models.
 * Implementing a node to retrieve the specific model's states using Gazebo service and publishing them
 * using C++ might be a better choice.
 */

#include <iostream>
#include <string>
#include <vector>

#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

class ModelStatePub
{
  private:
    int uav_num_;
    ros::NodeHandle nh_;
    bool pub_tf_;
    float pub_freq_;
    std::string state_source_;
    bool state_updated_;
    int *model_name_idx_map_ = nullptr; // array that maps the model name to the index of the model_states message.
                                        // array elements are the cooresponding index of the model_states message.
    bool all_models_ready_ = false;
    bool got_initial_pose_flag_ = false;

    ros::ServiceClient client_;
    ros::Subscriber model_state_sub_;
    ros::ServiceClient model_property_client_;
    ros::Publisher model_ready_pub_;
    std::vector<ros::Publisher> initial_pose_pub_;
    std::vector<ros::Publisher> pose_pubs_;
    std::vector<ros::Publisher> twist_pubs_;
    std::vector<geometry_msgs::PoseStamped> curr_pose_;
    std::vector<geometry_msgs::TwistStamped> curr_twist_;
    tf::TransformBroadcaster tf_pub_;

  public:
    ModelStatePub(ros::NodeHandle &nh)
    {
        nh_ = nh;
        state_updated_ = false;
        nh_.param<int>("uav_num", uav_num_, 9);
        nh_.param<bool>("pub_tf", pub_tf_, true);
        nh_.param<float>("pub_freq", pub_freq_, 60.0);
        nh_.param<std::string>("state_source", state_source_, "topic"); // service or topic
        std::cout << std::left << std::setw(10) << "\033[31mstate_source: \033[0m" << state_source_ << std::endl;
        std::cout << std::left << std::setw(10) << "\033[31muav_num: \033[0m" << uav_num_ << std::endl;
        std::string cout_str = pub_tf_ ? "true" : "false";
        std::cout << std::left << std::setw(10) << "\033[31mpub_tf: \033[0m" << cout_str << std::endl;
        std::cout << std::left << std::setw(10) << "\033[31mpub_freq: \033[0m " << pub_freq_ << std::endl;
        std::cout << std::string(50, '-') << std::endl;

        // subs
        if (state_source_ == "topic")
        {
            model_state_sub_ = nh_.subscribe("/gazebo/model_states", 1, &ModelStatePub::model_state_callback, this);
        }
        else if (state_source_ == "service")
        {
            client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        }

        // pubs
        model_property_client_ = nh_.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
        model_ready_pub_ = nh_.advertise<std_msgs::Bool>("/all_models_ready", 1, true);
        for (int i = 0; i < uav_num_; i++)
        {
            std::string topic_name = "/iris_" + std::to_string(i) + "/real_pose";
            // std::string topic_name = "/iris_" + std::to_string(i) + "/mavros/vision_pose/pose";
            pose_pubs_.push_back(nh_.advertise<geometry_msgs::PoseStamped>(topic_name, 1));
            topic_name = "/iris_" + std::to_string(i) + "/initial_pose";
            initial_pose_pub_.push_back(nh_.advertise<geometry_msgs::Pose>(topic_name, 1, true));
            topic_name = "/iris_" + std::to_string(i) + "/real_twist";
            twist_pubs_.push_back(nh_.advertise<geometry_msgs::TwistStamped>(topic_name, 1));
            curr_pose_.push_back(geometry_msgs::PoseStamped());
            curr_twist_.push_back(geometry_msgs::TwistStamped());
        }
    }

    ~ModelStatePub()
    {
        delete[] model_name_idx_map_;
    }

    void model_state_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        if (!all_models_ready_)
        {
            return;
        }
        /** @note This mapping technique optimizes name index retrieval to O(1), disregarding model additions and
         * deletions. Comment the code based on the application scenario.
         */
        if (model_name_idx_map_ == nullptr)
        {
            model_name_idx_map_ = new int[uav_num_];
            for (int i = 0; i < msg->name.size(); i++)
            {
                int idx = msg->name[i].find("iris_");
                if (idx == -1)
                {
                    continue;
                }
                int uav_idx = std::stoi(std::string(1, msg->name[i][5]));
                model_name_idx_map_[uav_idx] = i;
            }
        }
        for (int i = 0; i < uav_num_; i++)
        {
            std::string model_name = "iris_" + std::to_string(i);
            int idx_in_msg = model_name_idx_map_[i];

            curr_pose_[i].pose = msg->pose[idx_in_msg];
            curr_pose_[i].header.stamp = ros::Time::now();
            curr_pose_[i].header.frame_id = model_name + "/base_link";
            curr_twist_[i].twist = msg->twist[idx_in_msg];
            curr_twist_[i].header.stamp = ros::Time::now();
            curr_twist_[i].header.frame_id = model_name + "/base_link";
            if (!got_initial_pose_flag_)
            {
                initial_pose_pub_[i].publish(curr_pose_[i].pose);
            }
            // ROS_INFO("x: %f, y: %f, z: %f", curr_pose.pose.position.x, curr_pose.pose.position.y,
            //          curr_pose.pose.position.z);
        }
        got_initial_pose_flag_ = true;
        state_updated_ = true;
    }

    void run()
    {
        ros::Rate loop_rate(pub_freq_);
        ROS_INFO("model_pose_node started.");
        while (ros::ok())
        {
            if (!all_models_ready_)
            {
                int model_found_cnt = 0;
                for (int i = 0; i < uav_num_; i++)
                {
                    std::string model_name = "iris_" + std::to_string(i);
                    gazebo_msgs::GetModelProperties srv;
                    srv.request.model_name = model_name;
                    if (model_property_client_.call(srv))
                    {
                        if (srv.response.success)
                        {
                            model_found_cnt++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    ros::Duration(0.5).sleep();
                }
                if (model_found_cnt == uav_num_)
                {
                    all_models_ready_ = true;
                    ROS_INFO("All models are ready.");
                    model_ready_pub_.publish(true);
                }
                else
                {
                    ROS_INFO_THROTTLE(1, "Waiting for all models to be ready.");
                    model_ready_pub_.publish(false);
                    continue;
                }
            }
            if (state_source_ == "service")
            {
                for (int i = 0; i < uav_num_; i++)
                {
                    gazebo_msgs::GetModelState srv;

                    std::string model_name = "iris_" + std::to_string(i);
                    srv.request.model_name = model_name;
                    if (client_.call(srv))
                    {
                        curr_pose_[i].pose = srv.response.pose;
                        curr_pose_[i].header.stamp = ros::Time::now();
                        curr_pose_[i].header.frame_id = model_name + "/base_link";
                        curr_twist_[i].twist = srv.response.twist;
                        curr_twist_[i].header.stamp = ros::Time::now();
                        curr_twist_[i].header.frame_id = model_name + "/base_link";

                        if (!got_initial_pose_flag_)
                        {
                            initial_pose_pub_[i].publish(curr_pose_[i].pose);
                        }
                        // ROS_INFO("x: %f, y: %f, z: %f", curr_pose.pose.position.x, curr_pose.pose.position.y,
                        //          curr_pose.pose.position.z);
                    }
                }
                got_initial_pose_flag_ = true;
                state_updated_ = true;
            }

            if (state_updated_)
            {
                for (int i = 0; i < uav_num_; i++)
                {
                    pose_pubs_[i].publish(curr_pose_[i]);
                    twist_pubs_[i].publish(curr_twist_[i]);
                    if (pub_tf_)
                    {
                        tf_pub_.sendTransform(tf::StampedTransform(
                            tf::Transform(
                                tf::Quaternion(curr_pose_[i].pose.orientation.x, curr_pose_[i].pose.orientation.y,
                                               curr_pose_[i].pose.orientation.z, curr_pose_[i].pose.orientation.w),
                                tf::Vector3(curr_pose_[i].pose.position.x, curr_pose_[i].pose.position.y,
                                            curr_pose_[i].pose.position.z)),
                            ros::Time::now(), "world", "iris_" + std::to_string(i) + "/base_link"));
                    }
                }
                // state_updated_ = false;
            }
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_pose_node");
    ros::NodeHandle nh("~");
    ModelStatePub model_state_pub(nh);
    model_state_pub.run();
    return 0;
}
