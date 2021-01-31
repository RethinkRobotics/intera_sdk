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
from intera_motion_msgs.msg import WaypointOptions
from copy import deepcopy
from utility_functions import (
    ensure_path_to_file_exists,
    clamp_float_warn
)
from rospy_message_converter import message_converter
import yaml

class MotionWaypointOptions(object):
    """
    This class is a wrapper for the intera message:  WaypointOptions.msg
    It's primary purpose is to facilitate message creation and input checking.
    """

    # default Sawyer parameters
    default_speed_ratio = 0.7
    default_joint_tolerance = 0.05  # rad
    default_max_linear_speed = 0.6  # m/s
    default_max_linear_accel = 0.6  # m/s/s
    default_max_rot_speed = 1.57  # rad/s
    default_max_rot_accel = 1.57  # rad/s/s

    @staticmethod
    def get_accel_preset(accel_name):
        """
        Get the motion presets for max joint acceleration.
        This does not need to be set for Cartesian Trajectories.
        @return list [float]: limits from joints (j0 ... j6)
        """
        if accel_name == 'slow':
            return [1.5, 1.5, 3.0, 3.0, 3.0, 3.0, 3.0]
        if accel_name == 'medium':
            return [3.5, 2.5, 5.0, 5.0, 5.0, 5.0, 5.0]
        if accel_name == 'fast':
            return [7.0, 5.0, 8.0, 8.0, 8.0, 8.0, 8.0]
        if accel_name == 'express':
            return [10.0, 8.0, 10.0, 10.0, 12.0, 12.0, 12.0]
        rospy.logwarn('Did not recognize accel preset. Skipping.')
        return None

    @staticmethod
    def get_speed_ratio_preset(speed_name):
        if speed_name == 'slow':
            return 0.25
        if speed_name == 'medium':
            return 0.6
        if speed_name == 'fast':
            return 0.9
        if speed_name == 'express':
            return 1.0
        rospy.logwarn('Did not recognize speed ratio preset. Skipping.')
        return None

    def __init__(self, n_dim = 0,
                 label = "default",
                 max_joint_speed_ratio = None,
                 joint_tolerances = None,
                 max_joint_accel = None,
                 max_linear_speed = None,
                 max_linear_accel = None,
                 max_rotational_speed = None,
                 max_rotational_accel= None,
                 corner_distance = 0.0):
        """
        Create a motion waypoint options object. All parameters are
        optional. If ommitted or set to None, then use default value.
        See setter functions for more details.

        The last four parametes only apply to Cartesian trajectories.

        @param n_dim: Number of dimensions (joints) for the point.
        @param label: waypoint name
        @param max_joint_speed_ratio: Scales the maximum joint speed vector.
            If empty list, set to empty
        @param joint_tolerances: joint error tolerance
        @param max_joint_accel: max joint acceleration.
            If empty list, set to empty
        @param max_linear_speed: max linear speed
        @param max_linear_accel: max linear acceleration
        @param max_rotational_speed: max rotational speed
        @param max_rotational_accel: max rotational accceleration
        """

        self._n_dim = n_dim
        self._data = WaypointOptions()

        self.set_max_joint_speed_ratio(max_joint_speed_ratio)
        self.set_joint_tolerances(joint_tolerances)
        self.set_max_linear_speed(max_linear_speed)
        self.set_max_linear_accel(max_linear_accel)
        self.set_max_rotational_speed(max_rotational_speed)
        self.set_max_rotational_accel(max_rotational_accel)
        self.set_max_joint_accel(max_joint_accel)
        self.set_label(label)
        self.set_corner_distance(corner_distance)

    def set_max_joint_speed_ratio(self, speed_ratio = None):
        """
        The waypoint max joint speed is set to a percentage of the physical
        max joint speed. Cartesian paths should be set to 1.0
        @param speed_ratio: ratio [0.0, 1.0]
        """
        if speed_ratio is None:
            speed_ratio = self.default_speed_ratio
        speed_ratio = clamp_float_warn(0.05, speed_ratio, 1.0, 'speed_ratio')
        if speed_ratio is None:
            return
        self._data.max_joint_speed_ratio = speed_ratio

    def set_joint_tolerances(self, tol = []):
        """
        @param joint_tolerances:
        The error tolerance (radians) before the trajcectory is slowed
          - None:  populate with vector of default values
          - []: set tolerance vector to empty
          - float:  set every element in the joint_tolerance to this values
          - [float]:  copy all elements of joint_tolerances. Checks length.
        """
        if tol is None:
            self.set_joint_tolerances(self.default_joint_tolerance)
        elif not tol:
            self._data.tol = []
        elif isinstance(tol, float):
            tol = clamp_float_warn(1e-6, tol, float('inf'), 'joint tolerance')
            self._data.joint_tolerances = [tol] * self._n_dim
        elif len(tol) == self._n_dim:
            self._data.joint_tolerances = deepcopy(joint_tolerances)
        else:
            rospy.logerr('Invalid input to set_joint_tolerances!')

    def set_max_linear_speed(self, speed = None):
        """
        For Cartesian paths only
        @param speed: max linear tip speed (m/s)
        """
        if speed is None:
            speed = self.default_max_linear_speed
        tol = clamp_float_warn(0.001, speed, float('inf'), 'max linear speed')
        self._data.max_linear_speed = speed

    def set_max_linear_accel(self, accel = None):
        """
        For Cartesian paths only
        @param accel: max linear tip acceleration (m/s/s)
        """
        if accel is None:
            accel = self.default_max_linear_accel
        tol = clamp_float_warn(0.001, accel, float('inf'), 'max linear accel')
        self._data.max_linear_accel = accel

    def set_max_rotational_speed(self, speed = None):
        """
        For Cartesian paths only
        @param speed: max rotational tip speed (rad/s)
        """
        if speed is None:
            speed = self.default_max_rot_speed
        tol = clamp_float_warn(0.001, speed, float('inf'), 'max rotational speed')
        self._data.max_rotational_speed = speed

    def set_max_rotational_accel(self, accel = None):
        """
        For Cartesian paths only
        @param accel: max rotational tip acceleration (rad/s/s)
        """
        if accel is None:
            accel = self.default_max_rot_accel
        tol = clamp_float_warn(0.001, accel, float('inf'), 'max rotational accel')
        self._data.max_rotational_accel = accel

    def set_max_joint_accel(self, max_joint_accel = []):
        """
        @param max_joint_accel:
        Max joint acceleration per joint (rad/s/s)
          - None:  populate with vector of default values
          - []: set tolerance vector to empty
          - float:  scale the maximum acceleration for each joint by this
          - [float]:  copy all elements of max_joint_accel. Checks length.
          - str:  use a motion preset
        """
        if max_joint_accel is None:
            self.set_max_joint_accel('fast')
        elif not max_joint_accel:
            self._data.max_joint_accel = []
        elif isinstance(max_joint_accel, float):
            max_acc = self.get_accel_preset('express')
            max_joint_accel = clamp_float_warn(0.001, max_joint_accel, 1.0, 'joint accel')
            for i in range(0, self._n_dim):
                max_acc[i] = max_acc[i]*max_joint_accel
            self.set_max_joint_accel(max_acc)
        elif isinstance(max_joint_accel, str):
            self.set_max_joint_accel(self.get_accel_preset(max_joint_accel))
        else:  # Vector of doubles!
            self._data.max_joint_accel = max_joint_accel
            if not self.check_array_consistency():
                self._data.max_joint_accel = []


    def set_label(self, label="default"):
        """
        param label: name of waypoint
        """
        if isinstance(label, str):
            self._data.label = deepcopy(label)
        else:
            rospy.logerr('Input must be a string!')

    def set_corner_distance(self, corner_distance = None):
        """
        For Cartesian paths only
        @param corner_distance: Used for smoothing corners for continuous motion (meters)

        The distance from the waypoint to where the curve starts while blending from
        one straight line segment to the next.
        Larger distance:  trajectory passes farther from the waypoint at a higher speed
        Smaller distance:  trajectory passes closer to the waypoint at a lower speed
        Zero distance:  trajectory passes through the waypoint at zero speed
        """
        if corner_distance is None:
            corner_distance = 0.0;
        corner_distance = clamp_float_warn(0.0, corner_distance, 0.5, 'corner_distance')
        if corner_distance is None:
            return
        self._data.corner_distance = corner_distance

    def check_array_consistency(self):
        """
        @return: true iff all arrays in the data structure are self consistent.
        @rtype: bool
        """
        a = len(self._data.joint_tolerances)
        b = len(self._data.max_joint_accel)
        if a == 0 or b == 0 or a == b:
            return True
        else:
            rospy.logerr('Inconsistent array length in WaypointOptions!')
            return False

    def to_msg(self):
        """
        @return: MotionWaypoint.msg
        """
        return deepcopy(self._data)

    def to_dict(self):
        """
        @return: the waypoint options as a dictionary object
        """
        return message_converter.convert_ros_message_to_dictionary(self._data)

    def to_string(self):
        """
        @return: a yaml-formatted string with the waypoint options
        """
        return yaml.dump(self.to_dict(), default_flow_style=False)

    def to_yaml_file(self, file_name):
        """
        Write the contents of the waypoint options to a yaml file
        @param file_name: location to write file. Will create directory if needed.
        """
        file_name = ensure_path_to_file_exists(file_name)
        with open(file_name, "w") as outfile:
            yaml.dump(self.to_dict(), outfile, default_flow_style=False)
