import numpy as np
import quaternion
import warnings
warnings.filterwarnings("ignore", message=".*The 'nopython' keyword.*")
from geometry_msgs.msg import PoseStamped, Pose

def orientation_2_quaternion(orientation):
    return np.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)

def position_2_array(position):
    return np.array([position.x, position.y, position.z])

def pose_2_transformation(pose: Pose):
    quaternion_orientation = orientation_2_quaternion(pose.orientation)
    translation = position_2_array(pose.position)
    rotation_matrix = quaternion.as_rotation_matrix(quaternion_orientation)
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3, 0:3] = rotation_matrix
    transformation_matrix[0:3, 3] = translation
    return transformation_matrix

def pos_quat_2_pose_st(pos_array, quat):
    pose_st = PoseStamped()
    pose_st.pose.position.x = pos_array[0]
    pose_st.pose.position.y = pos_array[1]
    pose_st.pose.position.z = pos_array[2]
    pose_st.pose.orientation.x = quat.x
    pose_st.pose.orientation.y = quat.y
    pose_st.pose.orientation.z = quat.z
    pose_st.pose.orientation.w = quat.w
    return pose_st

def transformation_2_pose(transformation_matrix):
    pos_array = transformation_matrix[0:3, 3]
    rotation_matrix = transformation_matrix[0:3, 0:3]
    quat = quaternion.from_rotation_matrix(rotation_matrix)
    pose_st = pos_quat_2_pose_st(pos_array, quat)
    return pose_st

def pose_st_2_transformation(pose_st: PoseStamped):
    transformation_matrix = pose_2_transformation(pose_st.pose)
    return transformation_matrix

def transform_pose(pose: PoseStamped, transformation_matrix):
    pose_as_matrix = pose_st_2_transformation(pose)
    transformed_pose_matrix = transformation_matrix @ pose_as_matrix
    transformed_pose = transformation_2_pose(transformed_pose_matrix)
    return transformed_pose

def transform_pos_ori(pos: np.array, ori, transform):
    ori_quat = list_2_quaternion(ori)
    ori_rot_matrix = quaternion.as_rotation_matrix(ori_quat)
    transformed_ori_rot_matrix = transform[:3,:3] @ ori_rot_matrix
    pos = np.hstack((pos, 1))
    transformed_pos = transform @ pos
    transformed_ori_quat = quaternion.from_rotation_matrix(transformed_ori_rot_matrix)
    transformed_ori_array = np.array([transformed_ori_quat.w, transformed_ori_quat.x, transformed_ori_quat.y, transformed_ori_quat.z])
    return transformed_pos[:3], transformed_ori_array

def list_2_quaternion(quaternion_list: list):
    return np.quaternion(quaternion_list[0], quaternion_list[1], quaternion_list[2], quaternion_list[3])

def transform_between_poses(pose2: PoseStamped, pose1: PoseStamped):
    pose1_matrix = pose_st_2_transformation(pose1)
    pose2_matrix = pose_st_2_transformation(pose2)
    transform=pose2_matrix @ np.linalg.inv(pose1_matrix)
    return transform

def interpolate_poses(pose_start: PoseStamped, pose_goal: PoseStamped, interp_dist_linear, interp_dist_polar):
    position_goal = np.array([pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z])
    position_start = np.array([pose_start.pose.position.x, pose_start.pose.position.y, pose_start.pose.position.z])
    dist_lin = np.sqrt(np.sum(np.subtract(position_start, position_goal)**2))
    # dist = np.sqrt(np.sum(np.subtract(start, goal_array)**2, axis=0))
    
    step_num_lin = int(np.ceil(dist_lin / interp_dist_linear))
    try:
        quaternion_start = list_2_quaternion([pose_start.pose.orientation.w, pose_start.pose.orientation.x, pose_start.pose.orientation.y, pose_start.pose.orientation.z])
        quaternion_goal = list_2_quaternion([pose_goal.pose.orientation.w, pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z])

        quaternion_start_norm = np.sqrt(quaternion_start.x**2 + quaternion_start.y**2 + quaternion_start.z**2 + quaternion_start.w**2)
        quaternion_goal_norm = np.sqrt(quaternion_goal.x**2 + quaternion_goal.y**2 + quaternion_goal.z**2 + quaternion_goal.w**2)

        inner_prod= quaternion_start.x * quaternion_goal.x + quaternion_start.y * quaternion_goal.y + quaternion_start.z * quaternion_goal.z + quaternion_start.w * quaternion_goal.w
        if inner_prod < 0: quaternion_start = -quaternion_start
        inner_prod= quaternion_start.x * quaternion_goal.x + quaternion_start.y * quaternion_goal.y + quaternion_start.z * quaternion_goal.z + quaternion_start.w * quaternion_goal.w
        inner_prod= inner_prod / (quaternion_start_norm * quaternion_goal_norm)
        theta= np.arccos(np.abs(inner_prod))
    
        step_num_polar = int(np.ceil(theta / interp_dist_polar))
    except Exception as e:
        print("Zero division error in polar interpolation")
        print(f"Exception {e}")
        step_num_polar = 2
    
    step_num=np.max([2,np.max([step_num_polar,step_num_lin]) + 1])
    
    x = np.linspace(position_start[0], position_goal[0], step_num)
    y = np.linspace(position_start[1], position_goal[1], step_num)
    z = np.linspace(position_start[2], position_goal[2], step_num)

    poses = []
    for i in range(step_num):
        pos=np.array([x[i], y[i], z[i]])
        quat=np.slerp_vectorized(quaternion_start, quaternion_goal, i/(step_num-1))
        pose_st = pos_quat_2_pose_st(pos, quat)
        poses.append(pose_st)

    return poses
