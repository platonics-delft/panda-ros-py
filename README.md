# Interace of Python for the Human-Friendly Impedance Controller

This repository has many python classes that are used to control the panda in python using ROS. 
This is a ROS package, hence if you place it in your repository, it will be installed in your ROS workspace. 
This is the python interface to the robot that is controlled with the impedance controller available at 
https://github.com/franzesegiovanni/franka_human_friendly_controllers. 
Follow the instructions in the repository to install the cartesian impedance controller. You can use the class panda here to read the state of the robot and to send commands to the robot. 

After the installation of the package in your workspace, i.e, 

```
catkin build
source devel/setup.bash
```

The panda class can be accesses as: 
```
from panda_ros import Panda
panda=Panda()
```

This class allows to read the robot state, for example,

```
panda.curr_pos # current cartesian position 
panda.curr_ori # current cartesian orientation
panda.curr_joint # current joint position
```
You can set the cartesian attractor as: 

```
panda.goal_pub.publish(goal) # where goal is a PoseStamped message
```
or the desired joint configuration acting on the nullspace of the cartesian position: 
```
panda.set_configuration(joint_goal) # where joint_goal is an array or list of joint positions

```
You can send the robot to a particular pose with a linear trajectory using the following command:

```
panda.go_to_pose(goal) #where goal is a PoseStamped message

```

To ensure that the trajecotry is following a feasible solution in joint space, you can also have a joint trajectory in the null-space of the cartesian position: 

```

panda.go_to_pose_ik(goal) #where goal is a PoseStamped messag
```

the alogithm will compute an inverse kineamatics solution that correspond to the final goal. 

Sometimes the impedance controller is not precise enough, and you may want to compesate from some error when you reach a poisition

```
panda.offset_compensator(num_steps)
```

where num_steps is the number of attractor changes that the robot does to minimize the desired and the measured cartesian position and orientation. 


Have fun, and don't forget to star the repo and the human friendly controller repository. 
