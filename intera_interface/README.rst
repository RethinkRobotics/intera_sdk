intera_interface
================

Python interface classes and action servers for control of
the Baxter Research Robot from Rethink Robotics

Code & Tickets
--------------

+-----------------+----------------------------------------------------------------+
| Documentation   | http://sdk.rethinkrobotics.com/wiki                            |
+-----------------+----------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/intera_interface/issues     |
+-----------------+----------------------------------------------------------------+
| Contributions   | http://sdk.rethinkrobotics.com/wiki/Contributions              |
+-----------------+----------------------------------------------------------------+

intera_interface Repository Overview
------------------------------------

::

     .
     |
     +-- src/                                  intera_interface api
     |   +-- io_interface/                     basic interface for IO Framework
     |   +-- intera_interface/                 intera component classes
     |       +-- camera.py
     |       +-- cuff.py
     |       +-- digital_io.py
     |       +-- gripper.py
     |       +-- head.py
     |       +-- head_display.py
     |       +-- limb.py
     |       +-- navigator.py
     |       +-- robot_enable.py
     |       +-- robot_params.py
     |       +-- settings.py
     |   +-- intera_control/                   generic control utilities
     |   +-- intera_dataflow/                  timing/program flow utilities
     |   +-- joint_trajectory_action/          joint trajectory action implementation
     |   +-- gripper_action/                   gripper action implementation
     |   +-- head_action/                      head action implementation
     |
     +-- scripts/                              action server executables
     |   +-- enable_robot.py
     |   +-- gripper_action_server.py
     |   +-- head_action_server.py
     |   +-- joint_trajectory_action_server.py
     |
     +-- cfg/                                  dynamic reconfigure action configs


Other Intera Repositories
-------------------------

+------------------+-----------------------------------------------------+
| intera_common    | https://github.com/RethinkRobotics/intera_common    |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

http://sdk.rethinkrobotics.com/wiki/Release-Changes
