#! /usr/bin/env python

# Copyright (c) 2016-2018, Rethink Robotics Inc.
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
from geometry_msgs.msg import (
    PoseStamped,
    Pose
)
from intera_motion_msgs.msg import (
    Waypoint,
    WaypointOptions
)
from sensor_msgs.msg import JointState
from copy import deepcopy
from utility_functions import ensure_path_to_file_exists
from rospy_message_converter import message_converter
import yaml
from motion_waypoint_options import MotionWaypointOptions
from intera_interface import Limb

class MotionWaypoint(object):
    """
    This class is a wrapper for the intera message:  Waypoint.msg
    """

    @staticmethod
    def get_default_joint_angles():
        """
        @return: a set of joint angles corresponding to the sawyer arm
            directly in front of the robot with the elbow up.
        """
        return [0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0]

    @staticmethod
    def get_default_active_endpoint():
        """
        @return: the active endpoint  string corresponding to the tip of
            sawyer's arm when nothing else is attached.
        """
        return 'right_hand'

    def __init__(self, joint_angles = [],
                 active_endpoint = None,
                 options = None,
                 limb = None):
        """
        Create a MotionWaypoint object.  All parameters are optional.
        @param joint_angles: the joint angles to store in the waypoint.
            If set to empty, then create an empty waypoint.
            If set to None, then use the default joint angles.
        @param active_endpoint: the pose is computed using the active endpoint
            If set to None, then use default active endpoint
        @param options: waypoint options.
            If set to None, then use default waypoint options
        @param limb: Limb object
            if limb is None: create a new instance of Limb
        @return: a intera Waypoint.msg object with default values
        """
        self._data = Waypoint()
        self._limb = limb or Limb()

        self.set_joint_angles(joint_angles, active_endpoint)
        self.set_waypoint_options(options)

    def set_from_message(self, wpt_msg):
        """
        @param wpt_msg: Waypoint message
        """
        if isinstance(wpt_msg, Waypoint):
            self._data = deepcopy(wpt_msg)
        else:
            rospy.logerr('Failed to set waypoint from message: invalid type.')

    def set_waypoint_options(self, options = None):
        """
        Sets the options for this waypoint.
        @param options: A complete WaypointOptions message or
            MotionWaypointOptions object. If None: use default options
        """
        if options is None:
            n_dim = len(self._data.joint_positions)
            self.set_waypoint_options(MotionWaypointOptions(n_dim))
        elif isinstance(options, WaypointOptions):
            self._data.options = deepcopy(options)
        elif isinstance(options, MotionWaypointOptions):
            self._data.options = deepcopy(options.to_msg())
        else:
            rospy.logerr('Cannot set waypoint options: invalid instance type!')

    def set_angles_from_joint_state(self, joint_state):
        """
        @param joint_state: JointState object
        """
        if not isinstance(joint_state, JointState):
            rospy.logerr('Invalid argument: must be of type JointState')
        else:
            angles = joint_state.position
            names = joint_state.name
            self.set_joint_angles(joint_angles = angles)

    def get_joint_angles(self):
        """
        Get the waypoint joint angles
        @return: joint positions
        @rtype: [float]
        """
        return deepcopy(self._data.joint_positions)

    def set_joint_angles(self, joint_angles = [],
                         active_endpoint = None,
                         perform_fk = False):
        """
        All parameters are optional. If ommitted or set to None, use default.
        @param joint_angles: the joint angles to store in the waypoint.
            If set to empty, then create an empty waypoint.
        @param active_endpoint: the pose is computed using the active endpoint
        @param perform_fk: boolean to determine if FK should be performed with
            the input joint angles
        """
        if joint_angles is None:
            joint_angles = MotionWaypoint.get_default_joint_angles()
        if active_endpoint is None:
            active_endpoint = MotionWaypoint.get_default_active_endpoint()

        pose = PoseStamped()
        # If joint angles don't exist or we don't want to computer FK:
        if not perform_fk or not joint_angles:
            self._data.pose = pose   # empty pose as well
        else:   # Solve forward kinematics to get the pose
            joints = dict(zip(self._limb.joint_names(), joint_angles))
            pose.pose = self._limb.joint_angles_to_cartesian_pose(
                    joint_angles=joints,
                    end_point=active_endpoint)
            if pose is None:
                rospy.logerr('Failed to compute end effector pose!')
            self._data.pose = pose

        self._data.joint_positions = deepcopy(joint_angles)
        self._data.active_endpoint = deepcopy(active_endpoint)

    def set_cartesian_pose(self, pose = None,
                                 active_endpoint = None,
                                 joint_angles = []):
        """
        All parameters are optional. If ommitted or set to None, use default.
        @param pose: Cartesian position and orientation (PoseStamped msg)
            If set to empty, then create an empty waypoint.
        @param active_endpoint: the pose is computed using the active endpoint
        @param joint_angles: the joint angles to store in the waypoint.
            This is used as the nullspace bias if set.
        """
        if pose is None:
          self._data.pose = PoseStamped()  # empty pose
        else:
          self._data.pose = pose

        if active_endpoint is None:
            active_endpoint = MotionWaypoint.get_default_active_endpoint()
        self._data.active_endpoint = active_endpoint

        if joint_angles:
            self._data.joint_positions = joint_angles

    def to_msg(self):
        """
        @return: Waypoint.msg
        """
        return deepcopy(self._data)

    def to_dict(self):
        """
        @return: the data in the waypoint as a dictionary object
        """
        return message_converter.convert_ros_message_to_dictionary(self._data)

    def to_string(self):
        """
        @return: a yaml-formatted string with the waypoint data
        """
        return yaml.dump(self.to_dict(), default_flow_style=False)

    def to_yaml_file(self, file_name):
        """
        Write the contents of the waypoint to a yaml file
        @param file_name: location to write file. Will create directory if needed.
        """
        file_name = ensure_path_to_file_exists(file_name)
        with open(file_name, "w") as outfile:
            yaml.dump(self.to_dict(), outfile, default_flow_style=False)
