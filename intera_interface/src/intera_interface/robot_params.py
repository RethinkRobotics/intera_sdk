# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import socket

class RobotParams(object):
    """
    Interface class for essential ROS parameters on Intera robot.
    """

    def __init__(self):
        self._sdk_networking_url = "http://sdk.rethinkrobotics.com/intera/Networking"
        self._ros_networking_url = "http://wiki.ros.org/ROS/NetworkSetup"
        self._color_dict = {"INFO":'37',"WARN":'33',"ERROR":'31'}
        self._color_prefix = "\033[1;{0}m"
        self._color_suffix = "\033[1;m"

    def get_camera_names(self):
        """ Return the names of the camera.
        @rtype: list [str]
        @return: ordered list of camera names
        """
        return self.get_camera_details().keys()

    def get_camera_details(self):
        """
        Return the details of the cameras.

        @rtype: list [str]
        @return: ordered list of camera details
        """
        camera_dict = dict()
        camera_config_param = "/robot_config/camera_config"
        try:
            camera_dict = rospy.get_param(camera_config_param)
        except KeyError:
            rospy.logerr("RobotParam:get_camera_details cannot detect any "
                "cameras under the parameter {0}".format(camera_config_param))
        except (socket.error, socket.gaierror):
            self._log_networking_error()
        return  camera_dict

    def get_limb_names(self):
        """
        Return the names of the robot's articulated limbs from ROS parameter.

        @rtype: list [str]
        @return: ordered list of articulated limbs names
                 (e.g. right, left). on networked robot
        """
        non_limb_assemblies = ['torso', 'head']
        return list(set(self.get_robot_assemblies()).difference(non_limb_assemblies))

    def get_robot_assemblies(self):
        """
        Return the names of the robot's assemblies from ROS parameter.

        @rtype: list [str]
        @return: ordered list of assembly names
                 (e.g. right, left, torso, head). on networked robot
        """
        assemblies = list()
        try:
            assemblies = rospy.get_param("/robot_config/assembly_names")
        except KeyError:
            rospy.logerr("RobotParam:get_robot_assemblies cannot detect assembly names"
                         " under param /robot_config/assembly_names")
        except (socket.error, socket.gaierror):
            self._log_networking_error()
        return assemblies

    def get_joint_names(self, limb_name):
        """
        Return the names of the joints for the specified
        limb from ROS parameter.

        @type  limb_name: str
        @param limb_name: name of the limb for which to retrieve joint names

        @rtype: list [str]
        @return: ordered list of joint names from proximal to distal
                 (i.e. shoulder to wrist). joint names for limb
        """
        joint_names = list()
        try:
            joint_names = rospy.get_param(
                            "robot_config/{0}_config/joint_names".format(limb_name))
        except KeyError:
            rospy.logerr(("RobotParam:get_joint_names cannot detect joint_names for"
                          " arm \"{0}\"").format(limb_name))
        except (socket.error, socket.gaierror):
            self._log_networking_error()
        return joint_names

    def get_robot_name(self):
        """
        Return the name of class of robot from ROS parameter.

        @rtype: str
        @return: name of the class of robot (eg. "sawyer", "baxter", etc.)
        """
        robot_name = None
        try:
            robot_name = rospy.get_param("/manifest/robot_class")
        except KeyError:
            rospy.logerr("RobotParam:get_robot_name cannot detect robot name"
                         " under param /manifest/robot_class")
        except (socket.error, socket.gaierror):
            self._log_networking_error()
        return robot_name

    def _log_networking_error(self):
        msg = ("Failed to connect to the ROS parameter server!\n"
               "Please check to make sure your ROS networking is "
               "properly configured:\n"
               "Intera SDK Networking Reference: {0}\n"
               "ROS Networking Reference:        {1}").format(
                        self._sdk_networking_url,
                        self._ros_networking_url)
        self.log_message(msg, "ERROR")

    def log_message(self, msg, level="INFO"):
        """
        Print the desired message on stdout using
        level-colored text.

        @type  msg: str
        @param msg: Message text to be desplayed

        @type  level: str
        @param level: Level to color the text. Valid levels:
                      ["INFO", "WARN", "ERROR"]
        """
        print(("{0}[{1}] {2}{3}").format(
                        self._color_prefix.format(self._color_dict[level]),
                        level, msg, self._color_suffix))
