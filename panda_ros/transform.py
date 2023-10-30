import numpy as np
from panda_ros.pose_transform_functions import pose_st_2_transformation, array_quat_2_pose,  transform_pos_ori

class Transform():
    def __init__(self):
        super(Transform, self).__init__()

        # Home pose of panda_EE frame
        trans_home_pose = np.array([0.3067, 0.0007, 0.4840])
        quat_home_pose = np.quaternion(-0.0013368381495548441, 0.9999906149190223, -0.003484754266474329, 0.00016145904239296165)
        self.home_pose = array_quat_2_pose(trans_home_pose, quat_home_pose)


    def transform_traj_ori(self, traj, ori, transform):
        transformed_traj = np.empty_like(traj)
        transformed_ori = np.empty_like(ori)
        for i in range(traj.shape[1]):
            transformed_traj[:,i], transformed_ori[:,i] = transform_pos_ori(traj[:, i], ori[:, i], transform)
        return transformed_traj, transformed_ori

    def compute_final_transform(self):
        curr_pose= array_quat_2_pose(self.curr_pose, self.curr_ori)
        transform_new = pose_st_2_transformation(curr_pose)
        home_pose_matrix = pose_st_2_transformation(self.home_pose)
        self.final_transform =  transform_new @ np.linalg.inv(home_pose_matrix)
        print("final transform", self.final_transform)
        return self.final_transform
