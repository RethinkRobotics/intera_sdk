cmake_minimum_required(VERSION 2.8.3)
project(intera_interface)

find_package(catkin
  REQUIRED
  COMPONENTS
  cv_bridge
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/SawyerPositionJointTrajectoryActionServer.cfg
  cfg/SawyerVelocityJointTrajectoryActionServer.cfg
  cfg/SawyerPositionFFJointTrajectoryActionServer.cfg
)

add_dependencies(${PROJECT_NAME}_gencfg intera_core_msgs_generate_messages_py)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  rospy
  actionlib
  sensor_msgs
  std_msgs
  control_msgs
  trajectory_msgs
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
