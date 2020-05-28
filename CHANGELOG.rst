5.1.0 (2017-3-27)
---------------------------------
- Moved to Apache 2.0 license
- Added Contribution Guidelines
- Fix error logging in IK and FK scripts
- Adds default SDK head image to the repo for users to display
- Ensures "reset" command is available only after an estop or robot error
- Fixed issue with logic in gripper initialization and calibration

5.0.4 (2016-12-06)
---------------------------------
- Intera SDK is compatible with the Sawyer SDK Robot
- Initial release of intera_sdk intera_interface and intera_examples packages
- See intera_interface API Docs:
  https://rethinkrobotics.github.io/intera_sdk_docs/5.0.4/intera_interface/html/index.html
- Adds API classes for camera, cuff, gripper, head motion, head display, lights, limb,
  navigator, robot enabling, and robot parameters
- Adds scripts for calibration, enabling the robot, homing the sawyer joints, urdf fragment updating,
  and an entry point for the intera joint trajectory action server
- Adds intera_examples package with 19 code examples for interfacing with the robot
- Adds intera_sdk metapackage for intera_examples and intera_interface
