cmake_minimum_required(VERSION 3.0.2)
project(panda_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  franka_gripper
  franka_msgs
)

catkin_package()

# Make sure that your Python modules are available for import within your package
catkin_python_setup()

# Install the panda_ros package, including Python modules
install(DIRECTORY panda_ros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
