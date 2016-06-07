cmake_minimum_required(VERSION 2.8.3)
project(baxter_interface)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/PositionJointTrajectoryActionServer.cfg
  cfg/VelocityJointTrajectoryActionServer.cfg
  cfg/PositionFFJointTrajectoryActionServer.cfg
  cfg/GripperActionServer.cfg
  cfg/HeadActionServer.cfg
)

add_dependencies(${PROJECT_NAME}_gencfg baxter_core_msgs_generate_messages_py)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  baxter_core_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
