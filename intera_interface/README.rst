intera_interface
================

Python interface classes and action servers for control of
the Intera Research Robot from Rethink Robotics

Code & Tickets
--------------

+-----------------+----------------------------------------------------------------+
| Documentation   | http://sdk.rethinkrobotics.com/intera/                         |
+-----------------+----------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/intera_interface/issues     |
+-----------------+----------------------------------------------------------------+
| Contributions   | http://sdk.rethinkrobotics.com/intera/Contributions            |
+-----------------+----------------------------------------------------------------+

intera_interface Repository Overview
------------------------------------

::

     .
     |
     +-- src/                                  intera_interface api
     |   +-- intera_io/                        basic interface for IO Framework
     |   +-- intera_interface/                 intera component classes
     |       +-- camera.py
     |       +-- cuff.py
     |       +-- digital_io.py
     |       +-- gripper.py
     |       +-- head.py
     |       +-- head_display.py
     |       +-- lights.py
     |       +-- limb.py
     |       +-- navigator.py
     |       +-- robot_enable.py
     |       +-- robot_params.py
     |       +-- settings.py
     |   +-- intera_control/                   generic control utilities
     |   +-- intera_dataflow/                  timing/program flow utilities
     |   +-- intera_joint_trajectory_action/   joint trajectory action implementation
     |
     +-- scripts/                              utility executable scripts
     |   +-- calibrate_arm.py                  arm calibration action client
     |   +-- enable_robot.py                   enable / disable / reset the robot
     |   +-- home_joints.py                    script to home the joints on the robot
     |   +-- joint_trajectory_action_server.py trajectory action server for use with MoveIt
     |   +-- send_urdf_fragment.py             send URDF fragment to update robot's URDF
     |
     +-- cfg/                                  dynamic reconfigure action configs


Other Intera Repositories
-------------------------

+------------------+-----------------------------------------------------+
| intera_common    | https://github.com/RethinkRobotics/intera_common    |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

http://sdk.rethinkrobotics.com/intera/Release-Changes
