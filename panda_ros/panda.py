#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import warnings
warnings.filterwarnings("ignore", message=".*The 'nopython' keyword.*")
import quaternion # pip install numpy-quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from panda_ros.franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
from panda_ros.pose_transform_functions import  pos_quat_2_pose_st, list_2_quaternion, pose_2_transformation, interpolate_poses
from spatialmath import SE3 #pip install spatialmath-python
from spatialmath.base import q2r
import roboticstoolbox as rtb #pip install roboticstoolbox-python
class Panda():
    def __init__(self):
        super(Panda, self).__init__()

        self.K_pos=2000
        self.K_ori=30
        self.K_ns=30
        self.K_pos_safe=600
        self.K_ori_safe=10

        self.curr_pos=None
        self.curr_ori=None
        self.curr_pos_goal=None
        self.curr_ori_goal=None
        self.attractor_distance_threshold=0.05
        self.safety_check=True
        self.previous_safety_check=True
        self.change_in_safety_check=False

        
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)

        self.force_feedback_sub = rospy.Subscriber('/force_torque_ext', WrenchStamped, self.force_feedback_callback)
        self.goal_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, self.ee_pos_goal_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.configuration_pub = rospy.Publisher('/equilibrium_configuration', Float32MultiArray, queue_size=0)
        self.grasp_pub = rospy.Publisher("/franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.move_pub = rospy.Publisher("/franka_gripper/move/goal", MoveActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)

        self.vibration_pub = rospy.Publisher("/haptic_feedback", Float32, queue_size=0)
        
        self.force_feedback = 0.
        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
 
        self.move_command=MoveActionGoal()
        self.grasp_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.stop_command = StopActionGoal()
        self.gripper_width = 0
        self.move_command.goal.speed=1
        self.grasp_command.goal.epsilon.inner = 0.3
        self.grasp_command.goal.epsilon.outer = 0.3
        self.grasp_command.goal.speed = 0.1
        self.grasp_command.goal.force = 50
        self.grasp_command.goal.width = 1

        rospy.sleep(1)

    def ee_pos_goal_callback(self, goal_conf):
        self.goal_pose = goal_conf
        self.curr_pos_goal = np.array([goal_conf.pose.position.x, goal_conf.pose.position.y, goal_conf.pose.position.z])
        self.curr_ori_goal = np.array([goal_conf.pose.orientation.w, goal_conf.pose.orientation.x, goal_conf.pose.orientation.y, goal_conf.pose.orientation.z])
        self.safety_checker()
        
    def ee_pos_callback(self, curr_conf):
        self.curr_pose= curr_conf
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])

    def move_gripper(self,width):
        self.move_command.goal.width=width
        self.move_pub.publish(self.move_command)

    def grasp_gripper(self, width):
        self.grasp_command.goal.width = width
        self.grasp_pub.publish(self.grasp_command)

    def home(self, height=0.25, front_offset=0.4, side_offset=0.0):
        self.goal_pub.publish(self.curr_pose)
        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

        pos_array = np.array([front_offset, side_offset, height])
        quat = np.quaternion(0, 1, 0, 0)
        goal = pos_quat_2_pose_st(pos_array, quat)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()

        ns_msg = [0, 0, 0, -2.4, 0, 2.4, 0.8] #ensure that the elbow is upward
        self.go_to_pose_ik(goal, goal_configuration=ns_msg)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)  

    def force_feedback_callback(self, feedback):
        self.force = feedback.wrench.force
        self.force_feedback = np.linalg.norm(np.array([self.force.x, self.force.y, self.force.z]))

    def joint_states_callback(self, data):
        self.curr_joint = data.position[:7]
        self.gripper_width = data.position[7] + data.position[8]
    
    def set_configuration(self,joint):
        joint_des=Float32MultiArray()
        joint_des.data= np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)
    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        self.set_K.update_configuration({"translational_stiffness_X": k_t1})
        self.set_K.update_configuration({"translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({"translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        self.set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        self.set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        self.set_K.update_configuration({"nullspace_stiffness": k_ns}) 

    # control robot to desired goal position
    def go_to_pose(self, goal_pose: PoseStamped, interp_dist=0.001, interp_dist_polar=0.001): 
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        r = rospy.Rate(100)
        
        poses=  interpolate_poses(self.curr_pose, goal_pose, interp_dist, interp_dist_polar)
        for pose in poses:
            self.goal_pub.publish(pose)
            r.sleep()
        self.goal_pub.publish(goal_pose)    
        rospy.sleep(0.2)
    
        # control robot to desired goal position
    def go_to_pose_ik(self, goal_pose: PoseStamped, goal_configuration=None, interp_dist=0.001, interp_dist_joint=0.005): 
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        # self.set_K.update_configuration({"max_delta_lin": 0.2})
        # self.set_K.update_configuration({"max_delta_ori": 0.5}) 
        r = rospy.Rate(100)

        robot = rtb.models.Panda()
        position_start = self.curr_pos
        joint_start = np.array(self.curr_joint)
        goal_array = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])

        # interpolate from start to goal with attractor distance of approx 1 cm
        dist = np.sqrt(np.sum(np.subtract(position_start, goal_array)**2, axis=0))
        
        step_num_lin = math.floor(dist / interp_dist)
        q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
        if goal_configuration is None:
            quaternion_array = np.array([goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z]) 
            # normalize quaternion
            quaternion_array = quaternion_array / np.linalg.norm(quaternion_array)
            # Convert quaternion to rotation matrix
            rotation_matrix = q2r(quaternion_array)

            T = SE3.Rt(rotation_matrix, goal_array)

            # Solve inverse kinematics, try 5 times
            for i in range(5):
                # sol = robot.ikine_LM(T, q0=joint_start)
                sol = robot.ikine_LM(T,q0=joint_start)
                if sol.success:
                    goal_configuration = sol.q  # Joint configuration
                    print("Feasible joint configuration found")
                    break
            if not sol.success:
                for i in range(5):
                    sol = robot.ikine_LM(T)
                    if sol.success:
                        goal_configuration = sol.q  # Joint configuration
                        print("Feasible joint configuration found")
                        break

        # Check if the solution is valid
        if goal_configuration is not None:
             
            joint_distance = np.abs(np.subtract(joint_start, goal_configuration))
            max_joint_distance = np.max(joint_distance)
            step_num_joint = math.ceil(max_joint_distance / interp_dist_joint)
            # step_num_joint = int(np.ceil(np.linalg.norm(goal_configuration - joint_start) / interp_dist_joint))
            step_num=np.max([step_num_joint,step_num_lin])+1
        
            pos_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(position_start, [goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])]).T
            joint_goal = np.vstack([np.linspace(start, end, step_num) for start, end in zip(joint_start, goal_configuration)]).T

            # self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, 0, 0, 0, 0)
            rospy.sleep(0.2)
            self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, 0, 0, 0, self.K_ns)
            i=0
            while i < step_num:
                pose_goal = pos_quat_2_pose_st(pos_goal[i], q_goal)  
                self.goal_pub.publish(pose_goal)
                self.set_configuration(joint_goal[i])
                if self.safety_check:
                    i= i+1 
                
                if self.change_in_safety_check:
                    if self.safety_check:
                        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, 0, 0, 0, self.K_ns)
                        # self.set_K.update_configuration({"max_delta_lin": 0.2})
                        # self.set_K.update_configuration({"max_delta_ori": 0.5}) 
                    else:
                        self.set_stiffness(self.K_pos_safe, self.K_pos_safe, self.K_pos_safe, 0, 0, 0, 5)
                        # self.set_K.update_configuration({"max_delta_lin": 0.05})
                        # self.set_K.update_configuration({"max_delta_ori": 0.1})   
                r.sleep()
            self.set_stiffness(self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, 0)

            rospy.sleep(1) 

        else:
            print("No feasible joint configuration found or no joint configuration provided")
        
    def safety_checker(self):
        distance_pos = np.linalg.norm(self.curr_pos_goal - self.curr_pos)
        if distance_pos < self.attractor_distance_threshold:
            self.safety_check = True
        else:
            self.safety_check = False
        if self.safety_check != self.previous_safety_check:
            self.change_in_safety_check = True
        self.previous_safety_check = self.safety_check


    def offset_compensator(self, steps):
        curr_quat_desired= list_2_quaternion(np.copy(self.curr_ori_goal))
        curr_pos_desired = np.copy(self.curr_pos_goal )
        for _ in range(steps):
            curr_quat_goal= list_2_quaternion(self.curr_ori_goal)
            curr_pos_goal = self.curr_pos_goal 
            curr_quat = list_2_quaternion(self.curr_ori)    
            
                    
            quat_diff = curr_quat_desired * curr_quat.inverse() 
            lin_diff = curr_pos_desired - self.curr_pos 
            
            
            quat_goal_new = quat_diff * curr_quat_goal
            goal_pos = curr_pos_goal + lin_diff
            
            goal_pose = pos_quat_2_pose_st(goal_pos, quat_goal_new)
            self.goal_pub.publish(goal_pose) 
            rospy.sleep(0.2)
        
        
    def vibrate(self, duration: Float32):   
        self.vibration_pub.publish(duration)
