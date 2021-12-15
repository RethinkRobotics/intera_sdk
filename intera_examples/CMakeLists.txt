cmake_minimum_required(VERSION 2.8.3)
project(intera_examples)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/SawyerJointSpringsExample.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  intera_core_msgs
  intera_motion_msgs
  intera_interface
)

catkin_install_python(PROGRAMS 
  scripts/ik_service_client.py
  scripts/go_to_joint_angles.py
  scripts/joint_position_waypoints.py
  scripts/joint_torque_springs.py
  scripts/gripper_cuff_control.py
  scripts/head_wobbler.py
  scripts/joint_recorder.py
  scripts/fk_service_client.py
  scripts/joint_velocity_wobbler.py
  scripts/navigator_io.py
  scripts/joint_position_joystick.py
  scripts/joint_position_keyboard.py
  scripts/gripper_keyboard.py
  scripts/stop_motion_trajectory.py
  scripts/go_to_joint_angles_in_contact.py
  scripts/joint_position_file_playback.py
  scripts/go_to_cartesian_pose.py
  scripts/head_display_image.py
  scripts/joint_trajectory_client.py
  scripts/joint_trajectory_file_playback.py
  scripts/set_interaction_options.py
  scripts/constrained_zeroG.py
  scripts/gripper_joystick.py
  scripts/lights_blink.py
  scripts/camera_display.py
  scripts/send_random_trajectory.py
  scripts/send_traj_from_file.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
