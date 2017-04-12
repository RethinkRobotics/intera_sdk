#! /usr/bin/env python

# Copyright (c) 2016-2017, Rethink Robotics Inc.
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


import csv
import yaml
import rospy
import rosbag
import actionlib
from motion_msgs.msg import (
    Trajectory,
    TrajectoryOptions,
    Waypoint
)
import os.path
from intera_core_msgs.msg import (
    JointLimits
)
from sensor_msgs.msg import (
    JointState
)
from motion_interface.motion_controller_action_client import (
    MotionControllerActionClient
)
from motion_interface.motion_waypoint import MotionWaypoint
from motion_interface.motion_waypoint_options import MotionWaypointOptions
from motion_interface.utility_functions import ensure_path_to_file_exists
from copy import deepcopy
from rospy_message_converter import message_converter


class MotionTrajectory(object):

    @staticmethod
    def get_default_joint_names():
        """
        @return: a names for sawyer's primary joints
        """
        return ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                'right_j4', 'right_j5', 'right_j6']


    def __init__(self, label = None,
                 joint_names = None,
                 trajectory_options = None):
        """
        Creates a new motion trajectory object.
        @param label: string
            if label is None: use default
        @param joint_names: name for each joint in the robot
            if joint_names is None: use default names
        @param trajectory_options: options for the trajectory
            if trajectory_options is None: use default options
        """

        # Used for sending the trajectory to the motion controller
        self._client = MotionControllerActionClient()
        self._traj = Trajectory()

        self.set_label(label)
        self.set_joint_names(joint_names)
        self.set_trajectory_options(trajectory_options)

    def stop_trajectory(self):
        self._client.stop_trajectory()

    def send_trajectory(self):
        if not self._traj.waypoints:
            rospy.logerr("Trajectory is empty! Cannot send.")
            return None
        self._client.send_trajectory(self.to_msg())
        result = self._client.wait_for_result()
        return result

    def set_data(self, traj_msg):
        if isinstance(traj_msg, Trajectory):
            self._traj = deepcopy(traj_msg)
        else:
            rospy.logerr('Cannot set trajectory data. Invalid type.')

    def set_label(self, label=None):
        if label is None:
            label = 'default'
        self._traj.label = label

    def set_joint_names(self, joint_names = None):
        if joint_names is None:
            joint_names = self.get_default_joint_names()
        self._traj.joint_names = deepcopy(joint_names)

    def set_trajectory_options(self, trajectory_options = None):
        # TODO:  Better support for interaction control options.
        if trajectory_options is None:
            trajectory_options = TrajectoryOptions()
            trajectory_options.interpolation_type = TrajectoryOptions.JOINT
        if not isinstance(trajectory_options, TrajectoryOptions):
            rospy.logerr('Cannot append options: invalid instance type!')
        else:
            self._traj.trajectory_options = deepcopy(trajectory_options)

    def get_dimension(self):
        """
        @return: number of dimensions along the trajectory
        """
        return len(self._joint_names)

    def get_label(self):
        return self._traj.label

    def clear_waypoints(self):
        self._traj.waypoints = []

    def append_waypoint(self, waypoint):
        """
        Appends a waypoint to the trajectory message
        """
        if isinstance(waypoint, Waypoint):
            wpt = deepcopy(waypoint)
        elif isinstance(waypoint, MotionWaypoint):
            wpt = waypoint.to_msg()
        else:
            rospy.logerr('Invalid waypoint - cannot append to trajectory!')
            return False

        self._traj.waypoints.append(wpt)

    def get_waypoint_joint_angles_as_list(self):
        """
        @return: a list of lists, with the following format:
        [joint_names,waypoint_0,waypoint_1,...waypoint_N]
        """
        data = []
        data.append(self._traj.joint_names)
        for wpt in self._traj.waypoints:
            data.append(deepcopy(wpt.joint_positions))
        return data

    def to_msg(self):
        return deepcopy(self._traj)

    def to_dict(self):
        """
        @return: the waypoint options as a dictionary object
        """
        return message_converter.convert_ros_message_to_dictionary(self._traj)

    def to_string(self):
        """
        @return: a yaml-formatted string with the waypoint options
        """
        return yaml.dump(self.to_dict(), default_flow_style=False)

    def to_yaml_file(self, file_name):
        """
        Write the contents of the trajectory message to a yaml file
        """
        file_name = ensure_path_to_file_exists(file_name)
        with open(file_name, "w") as outfile:
            yaml.dump(self.to_dict(), outfile, default_flow_style=False)

    def to_csv_file(self, file_name):
        """
        Write the joint angles of the waypoints to a csv file. The first row
        contains the joint names and subsequent rows contain joint angles.
        """
        file_name = ensure_path_to_file_exists(file_name)
        waypoint_data = self.get_waypoint_joint_angles_as_list()
        with open(file_name, "wb") as f:
            writer = csv.writer(f)
            writer.writerows(waypoint_data)
