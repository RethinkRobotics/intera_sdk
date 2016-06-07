baxter_interface
================

Python interface classes and action servers for control of
the Baxter Research Robot from Rethink Robotics

Code & Tickets
--------------

+-----------------+----------------------------------------------------------------+
| Documentation   | http://sdk.rethinkrobotics.com/wiki                            |
+-----------------+----------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/baxter_interface/issues     |
+-----------------+----------------------------------------------------------------+
| Contributions   | http://sdk.rethinkrobotics.com/wiki/Contributions              |
+-----------------+----------------------------------------------------------------+

baxter_interface Repository Overview
------------------------------------

::

     .
     |
     +-- src/                                  baxter_interface api
     |   +-- baxter_interface/                 baxter component classes
     |       +-- analog_io.py
     |       +-- camera.py
     |       +-- digital_io.py
     |       +-- gripper.py
     |       +-- head.py
     |       +-- limb.py
     |       +-- navigator.py
     |       +-- robot_enable.py
     |       +-- robust_controller.py
     |       +-- settings.py
     |   +-- baxter_control/                   generic control utilities
     |   +-- baxter_dataflow/                  timing/program flow utilities
     |   +-- joint_trajectory_action/          joint trajectory action implementation
     |   +-- gripper_action/                   gripper action implementation
     |   +-- head_action/                      head action implementation
     |
     +-- scripts/                              action server executables
     |   +-- joint_trajectory_action_server.py
     |   +-- gripper_action_server.py
     |   +-- head_action_server.py
     |
     +-- cfg/                                  dynamic reconfigure action configs


Other Baxter Repositories
-------------------------

+------------------+-----------------------------------------------------+
| baxter           | https://github.com/RethinkRobotics/baxter           |
+------------------+-----------------------------------------------------+
| baxter_tools     | https://github.com/RethinkRobotics/baxter_tools     |
+------------------+-----------------------------------------------------+
| baxter_examples  | https://github.com/RethinkRobotics/baxter_examples  |
+------------------+-----------------------------------------------------+
| baxter_common    | https://github.com/RethinkRobotics/baxter_common    |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

http://sdk.rethinkrobotics.com/wiki/Release-Changes
