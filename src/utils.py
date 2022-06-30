"""
 # @ Author: Kenneth Simon
 # @ Email: smkk00715@gmail.com
 # @ Create Time: 2022-05-18 10:59:07
 # @ Modified time: 2022-05-18 11:33:28
 # @ Description:
 """


import ipdb
import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Pose, PoseArray


def get_near_pose(start_pose: Pose, target_pose: Pose, max_distance):
    pose_ar1 = pose_msg2ar(start_pose)
    pose_ar2 = pose_msg2ar(target_pose)
    vec = pose_ar2[:3] - pose_ar1[:3]
    vec_norm = np.linalg.norm(vec)
    if vec_norm <= max_distance:
        return target_pose
    else:
        vec = vec / vec_norm * max_distance + pose_ar1[:3]
        new_pose = set_pose_msg(Pose(), vec, pose_ar2[3:])
        return new_pose


def euclidean_distance(p1: Pose, p2: Pose):
    p1 = [p1.position.x, p1.position.y, p1.position.z]
    p2 = [p2.position.x, p2.position.y, p2.position.z]
    return np.linalg.norm(np.array(p1) - np.array(p2))


def linear_interpolation(start_pose: Pose, target_pose: Pose, max_distance=None, num_steps=10):
    if max_distance is not None:
        dist = euclidean_distance(start_pose, target_pose)
        num_steps = max(num_steps, int(dist / max_distance * num_steps))
    interpolated_poses = [start_pose]
    current_pose = start_pose
    target_pos = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    for step in range(1, num_steps + 1):
        current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        alpha = step / num_steps
        new_pos = current_pos + (target_pos - current_pos) * alpha
        new_quaternion = tft.quaternion_slerp(
            (
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ),
            (
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w,
            ),
            alpha,
        )
        new_pose = set_pose_msg(Pose(), new_pos, new_quaternion)
        interpolated_poses.append(new_pose)
        current_pose = new_pose
    return interpolated_poses


def pose_inverse(pose: Pose):
    pose_msg = Pose()
    pose_msg.position.x = -pose.position.x
    pose_msg.position.y = -pose.position.y
    pose_msg.position.z = -pose.position.z
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    quat = tft.quaternion_inverse(quat)
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    return pose_msg


def matrix_to_pose_msg(pose_matrix: np.array):
    """convert 4x4 homogeneous matix to Pose

    Args:
        pose_matrix (np.array): 4x4 homogeneous matrix
    """
    pose_msg = Pose()
    quat = tft.quaternion_from_matrix(pose_matrix)
    pos = pose_matrix[:3, -1]
    set_pose_msg(pose_msg, pos, quat)
    return pose_msg


def pose_msg_to_matrix(pose: Pose):
    """_summary_

    Returns:
        transformation_matrix: 4x4 transformation matrix
    """
    translation = np.array([pose.position.x, pose.position.y, pose.position.z, 1])
    rotation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    rotation_mat = tft.quaternion_matrix(rotation)
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_mat[:3, :3]
    transformation_matrix[:, 3] = translation
    return transformation_matrix


def apply_transformation_pose(pose1: Pose, pose2: Pose):
    """

    Returns:
        result_pose
    """
    transformation_matrix_1 = pose_msg_to_matrix(pose1)
    transformation_matrix_2 = pose_msg_to_matrix(pose2)
    result_matrix = tft.concatenate_matrices(transformation_matrix_1, transformation_matrix_2)
    pose_msg = Pose()
    set_pose_msg(pose_msg, tft.translation_from_matrix(result_matrix), tft.quaternion_from_matrix(result_matrix))
    return pose_msg


def apply_transformation_matrix(pos1: np.array, quat1: np.array, pos2: np.array, quat2: np.array):
    """_summary_

    Returns:
        result_matrix: 4x4 homogeneous matrix
    """
    translation_mat1 = tft.translation_matrix(pos1)
    rotation_mat1 = tft.quaternion_matrix(quat1)
    translation_mat2 = tft.translation_matrix(pos2)
    rotation_mat2 = tft.quaternion_matrix(quat2)
    result_matrix = tft.concatenate_matrices(translation_mat1, rotation_mat1, translation_mat2, rotation_mat2)
    return result_matrix


def set_pose_msg(pose_msg: Pose, pos, quat):
    pose_msg.position.x = pos[0]
    pose_msg.position.y = pos[1]
    pose_msg.position.z = pos[2]
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    return pose_msg


def set_pose_msg_orien(pose_msg: Pose, quat):
    pose_msg.orientation.x = quat[0]
    pose_msg.orientation.y = quat[1]
    pose_msg.orientation.z = quat[2]
    pose_msg.orientation.w = quat[3]
    return pose_msg


def pose_msg2ar(pose_msg: Pose, rtn_np=True):
    ar = [
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z,
        pose_msg.orientation.w,
    ]
    return np.array(ar, dtype=np.float32) if rtn_np else ar


def ENU2NED_Pose(pose_in_ENU: Pose):
    """transfer pose in ENU frame to NED frame

    pose_in_ENU: Pose

    return: Pose"""
    pose_in_NED = Pose()
    pose_in_NED.position.x = pose_in_ENU.position.y
    pose_in_NED.position.y = pose_in_ENU.position.x
    pose_in_NED.position.z = -pose_in_ENU.position.z
    pose_in_NED.orientation.x = pose_in_ENU.orientation.y
    pose_in_NED.orientation.y = pose_in_ENU.orientation.x
    pose_in_NED.orientation.z = -pose_in_ENU.orientation.z
    pose_in_NED.orientation.w = pose_in_ENU.orientation.w
    return pose_in_NED


def ENU2NED_Position(state_in_inertial):
    """transfer pose in ENU frame to NED frame

    state_in_NED:(x,y,z,vx,vy,vz)

    return: (x',y',z',vx',vy',vz')"""
    return np.array(
        [
            state_in_inertial[1],
            state_in_inertial[0],
            -state_in_inertial[2],
            state_in_inertial[4],
            state_in_inertial[3],
            -state_in_inertial[5],
        ]
    )


def NED2ENU_Position(state_in_NED):
    """transfer pose in NED frame to ENU frame
    state_in_NED:(x,y,z,vx,vy,vz)

    return: (x',y',z',vx',vy',vz')
    """
    return np.array(
        [state_in_NED[1], state_in_NED[0], -state_in_NED[2], state_in_NED[4], state_in_NED[3], -state_in_NED[5]]
    )


def format_dict(dictionary):
    max_key_length = max(len(key) for key in dictionary)
    formatted = ""
    for key, value in dictionary.items():
        if isinstance(value, np.ndarray):
            value_str = np.array2string(
                value, separator=", ", prefix=" " * max_key_length, formatter={"all": lambda x: f"{x}"}
            )
        else:
            value_str = str(value)
        formatted += f"{key:{max_key_length}}: {value_str}\n"
    return formatted


def thrust2signal(thrust: float, motor_constant, input_scale):
    """convert thrust(N) to PX4 throttle signal in [0,1]"""
    return np.sqrt((thrust + 0.0001) / motor_constant / 4) / input_scale


def inertial2body(vec, pose_in_interial):
    """_summary_

    Args:
        vec (List): 3-dim vector in inertial frame
        pose_in_interial (Pose): pose(quaternion) in interial frame

    Returns:
        List: 3-dim vector in body frame
    """
    [phi, theta, psi] = tft.euler_from_quaternion(
        [
            pose_in_interial.orientation.x,
            pose_in_interial.orientation.y,
            pose_in_interial.orientation.z,
            pose_in_interial.orientation.w,
        ]
    )
    [phi, theta, psi, _, _, _] = ENU2NED_Position([phi, theta, psi, 0, 0, 0])
    rotate_matrix = tft.euler_matrix(phi, theta, psi)
    vec_in_body = rotate_matrix @ np.array([[vec[0]], [vec[1]], [vec[2]], [1]])
    return [vec_in_body[0][0], vec_in_body[1][0], vec_in_body[2][0]]


def calcu_capacity(band_width, power, gain, noise_power):
    """calculate the capacity of a channel according to Shannon-Hartley theorem"""
    return band_width * np.log2(1 + power * gain / noise_power)


def dbm2watt(dbm):
    return 10 ** (dbm / 10) / 1000


def watt2dbm(watt):
    return 10 * np.log10(watt * 1000)


def friis_path_loss(d, f):
    """

    Args:
        d (float): distance to the transmitter
        f (float): frequency

    Returns:
        float: path loss in dB
    """
    return 20 * np.log10(d) + 20 * np.log10(f) + 20 * np.log10(4 * np.pi / 3e8)


def log_path_loss(L0, n, d0, d):
    """

    Args:
        L0 (float): path loss at reference distance
        n (float): loss distance exponent. 2~4 in countryside, 2~5 in city
        d (float): distance to the transmitter
        d0 (float): reference distance

    Returns:
        float: path loss in dB
    """
    return L0 + 10 * n * np.log10(d / d0)


def two_ray_ground_reflection(d, ht, hr, Gt, Gr):
    """calculate the path loss using two-ray ground reflection model

    Args:
        d (float): distance to the transmitter
        hr (float): height of the receiver
        ht (float): height of the transmitter

    Returns:
        float: path loss in dB
    """
    return 40 * np.log10(d) - 10 * np.log10(Gt * Gr * ht**2 * hr**2)


if __name__ == "__main__":
    if 0:  # test inertial2body
        pose = Pose()
        quat = tft.quaternion_from_euler(0, 0, np.radians(45))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        result = inertial2body([1, 0, 0], pose)
        # print(f"==>> result: {result}")
    if 0:  # test transformation
        pos1 = (1.0, 0.0, 0.0)
        quat1 = tft.quaternion_from_euler(0, 0, np.deg2rad(90))
        pos2 = (0.0, -1.0, 0.0)
        quat2 = (0.0, 0.0, 0, 1)
        transformation_matrix = apply_transformation_matrix(pos1, quat1, pos2, quat2)
        # print("Transformation Matrix:")
        # print(transformation_matrix)

        pose1 = Pose()
        pose2 = Pose()
        set_pose_msg(pose1, pos1, quat1)
        set_pose_msg(pose2, pos2, quat2)
        transformation_matrix = apply_transformation_pose(pose1, pose2)
        # print("Transformation Matrix:")
        # print(transformation_matrix)
        transformation_matrix = apply_transformation_pose(pose2, pose1)
        # print("Transformation Matrix:")
        # print(transformation_matrix)

    if 0:  # test linear interpolation
        rospy.init_node("utils_test")
        pose_pub = rospy.Publisher("test_linear_interpolation", PoseArray, queue_size=1)
        initial_pose = set_pose_msg(Pose(), [0, 0, 0], [0, 0, 0, 1])
        target_pose = set_pose_msg(Pose(), [5, 5, 1], tft.quaternion_from_euler(0, np.deg2rad(90), 0))
        interpolated_result = linear_interpolation(initial_pose, target_pose, num_steps=1, max_distance=None)
        pose_ar_msg = PoseArray()
        pose_ar_msg.poses = interpolated_result
        pose_ar_msg.header.frame_id = "world"

        print(f"==>> interpolated_result: {interpolated_result}")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose_pub.publish(pose_ar_msg)
            rate.sleep()
    if 1:
        pose1 = set_pose_msg(Pose(), [0, 0, 0], [0, 0, 0, 1])
        pose2 = set_pose_msg(Pose(), [5, 5, 5], [0, 0, 0, 1])
        near_pose = get_near_pose(pose1, pose2, max_distance=2)
        print(f"==>> near_pose: {near_pose}")
