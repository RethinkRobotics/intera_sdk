# Copyright (c) 2016, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import socket

class RobotParams(object):
    """
    Interface class for essential ROS parameters on Intera robot.
    """

    def __init__(self):
        self._sdk_networking_url = "http://sdk.rethinkrobotics.com/wiki/Networking"
        self._ros_networking_url = "http://wiki.ros.org/ROS/NetworkSetup"
        self._color_dict = {"INFO":'37',"WARN":'33',"ERROR":'31'}
        self._color_prefix = "\033[1;{0}m"
        self._color_suffix = "\033[1;m"

    def get_camera_names(self):
        """ Return the names of the camera.
        @rtype: [str]
        @return: ordered list of camera names
        """
        return self.get_camera_details().keys()

    def get_camera_details(self):
        """
        Return the details of the cameras.

        @rtype: [str]
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

        @rtype: [str]
        @return: ordered list of articulated limbs names
                 (e.g. right, left). on networked robot
        """
        limbs = list()
        non_limb_assemblies = ['torso', 'head']
        try:
            assemblies = rospy.get_param("/robot_config/assembly_names")
            limbs = [assembly for assembly in assemblies if assembly not in non_limb_assemblies]
        except KeyError:
            rospy.logerr("RobotParam:get_limb_names cannot detect limbs names"
                         " under param /robot_config/assembly_names")
        except (socket.error, socket.gaierror):
            self._log_networking_error()
        return limbs

    def get_joint_names(self, limb_name):
        """
        Return the names of the joints for the specified
        limb from ROS parameter.

        @type  limb_name: str
        @param limb_name: name of the limb for which to retrieve joint names

        @rtype: [str]
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
