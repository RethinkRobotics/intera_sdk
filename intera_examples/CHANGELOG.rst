1.2.0 (2015-12-21)
---------------------------------
- Added a URDF Fragment publishing example to demonstrate URDF additions to a Real Baxter
- The head_pan speed range has been changed to [0, 1.0]. Updated the examples & head action client accordingly
- Updated Digtial IO example to use the new light names for the navigator
- Remapped Joy stick example's Logitech controller to be similar to the Xbox controls
- Fixed a bug causing Gripper timing to lag in the joint playback example

1.1.1 (2015-5-15)
---------------------------------
- Added a calculation to increase the amount of time allowed to move arm to the
  initial pose of joint_trajectory_playback
- Fixed an issue in syncing gripper playback with joint_trajectory_playback arm execution
- Fixed a timing issue preventing joint_trajectory_playback from completing execution
- Removed incorrect internal tag for baxterworking.png

1.1.0 (2015-1-2)
---------------------------------
- Updates baxter_examples to ROS Indigo
- Updates xdisplay_image to properly publish images with opencv2
- Updates joint_trajectory_client to have a goal tolerance, and supplies current joint angles at time 0.0
- Updates joint_trajectory_file_playback to wait for the joint trajectory action server for 10.0 seconds
- Adds head_action_client example program

1.0.0 (2014-5-1)
---------------------------------
- Adds joint_position_waypoints example program
- Updates ik_service_client to validate unpacked results and seed types
- Updates gripper_cuff_control to automatically calibrate on gripper type change
- Updates navigator_io to use navigator wheel_changed signal
- Updates all examples to verify robot software version by default when enabling
- Updates all examples using gripper to verify gripper firmware version
- Updates joint_recorder to record at 100Hz
- Updates joint_velocity_wobbler to run at 500Hz

0.7.0 (2013-11-21)
---------------------------------
- Creation of baxter_examples repository from sdk-examples/examples.
- Adds joint torque springs examples.
- Adds gripper cuff control example.
- Adds gripper action client example.
- Package restructure in support of Catkin expected standards.
- Adds launch files for examples using action servers or the joystick.
- Adds gripper position playback to joint trajectory file playback example.
- Removes camera_control example, now located in baxter_tools repository.
- Removes getch usage as means of exiting example programs (latency).
- Fixes joint position file playback looping. Loops now start at correct playback start.
- Fixes head movement during exit of wobbler example.
- Adds timeouts to action client's wait_for calls making sure the action servers are running.
- Fixes D-Pad mapping for ps3 joysticks.
- Adds success verification for result of joint trajectory file playback.
