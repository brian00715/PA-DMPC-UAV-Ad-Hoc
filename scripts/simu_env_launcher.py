import roslaunch
import rospy


def launch_gazebo_empty_world():
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = [
        "$(find gazebo_ros)/launch/empty_world.launch",
        "gui:=true",
        f"world_name:=$(find dmpc_uav_ad_hoc)/worlds/disaster.world",
        "debug:=false",
        "verbose:=false",
        "paused:=false",
    ]

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    launch_args = cli_args[2:]  # Skip the roslaunch command and package name

    launch_args_parsed = [roslaunch.substitution.resolve_args(["$(arg {})".format(arg) for arg in launch_args])]

    roslaunch_file, launch_args = roslaunch.rlutil.split_launch_args(launch_args_parsed)

    roslaunch_parent = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_file], is_core=True)
    roslaunch_parent.start()

    return roslaunch_parent


def main():
    rospy.init_node("launch_from_python", anonymous=True)

    uav_num = rospy.get_param("~uav_num", 3)  # Default value is 3 if not provided

    gazebo_launch = launch_gazebo_empty_world()

    swarm_spawner_node = roslaunch.core.Node(
        package="dmpc_uav_ad_hoc",
        node_type="swarm_spawner.py",
        name="swarm_spawner",
        args=["uav_num:={}".format(uav_num)],
    )

    swarm_spawner_launch = roslaunch.scriptapi.ROSLaunch()
    swarm_spawner_launch.start()
    swarm_spawner_process = swarm_spawner_launch.launch(swarm_spawner_node)

    rospy.spin()

    swarm_spawner_process.stop()
    gazebo_launch.shutdown()


if __name__ == "__main__":
    main()
