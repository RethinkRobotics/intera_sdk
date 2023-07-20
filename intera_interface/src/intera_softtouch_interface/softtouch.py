#! /usr/bin/env python
# ****************************************************************************
# softtouch.py
# Created on Wed Jul 19 2023
# Author: Philipp Milfeit <p.milfeit@rethinkrobotics.com>
#
# Copyright (c) 2023, Rethink Robotics GmbH.
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
# ****************************************************************************

from copy import deepcopy

from intera_core_msgs.msg import EndpointState
from intera_motion_msgs.msg import SoftTouchCommandGoal

from .softtouch_action_client import SoftTouchActionClient
import rospy


class SoftTouch(object):
    """
    This class is a wrapper for the soft touch action:  SoftTouchCommand.action.

    The class creates an SoftTouchActionClient for sending
    messages to the SoftTouchActionServer and provides an interface for the user.
    """

    # Constants for default parameters
    TORQUE_LIMIT = 20.0
    FORCE_LIMIT = 20.0
    STANDARD_FRAME = 'base'
    MIN_LINEAR_SPEED = 0.001
    MAX_LINEAR_SPEED = 0.02
    MIN_ROTATIONAL_SPEED = 0.001
    MAX_ROTATIONAL_SPEED = 0.04
    LINEAR_GOAL_TOLERANCE = 0.01
    ROTATIONAL_GOAL_TOLERANCE = 0.01

    def __init__(self):
        """Create an action client for the Soft Touch."""
        # Used for sending the command to the action server
        rospy.init_node('soft_touch_client')
        self._client = SoftTouchActionClient()
        self._current_state = self.__get_current_state()
        self._desired_goal = self.__create_default_soft_touch_goal()

    def __get_current_state(self):
        """Get the current endpoint state."""
        try:
            return rospy.wait_for_message(
                '/robot/limb/right/endpoint_state', EndpointState, timeout=2
            )
        except rospy.ROSException:
            return None

    def __update_position(self, goal_to_update):
        """Update the goal and set the actual position."""
        self._current_state = self.__get_current_state()
        if self._current_state:
            goal_to_update.goal.position.x = self._current_state.pose.position.x
            goal_to_update.goal.position.y = self._current_state.pose.position.y
            goal_to_update.goal.position.z = self._current_state.pose.position.z
            goal_to_update.goal.orientation.x = self._current_state.pose.orientation.x
            goal_to_update.goal.orientation.y = self._current_state.pose.orientation.y
            goal_to_update.goal.orientation.z = self._current_state.pose.orientation.z
            goal_to_update.goal.orientation.w = self._current_state.pose.orientation.w

    def __create_default_soft_touch_goal(self):
        """Create an action goal with default parameters."""
        default_goal = SoftTouchCommandGoal()
        default_goal.header.stamp = rospy.get_rostime()
        default_goal.header.frame_id = self.STANDARD_FRAME
        default_goal.frame_name = self.STANDARD_FRAME
        default_goal.min_linear_speed = self.MIN_LINEAR_SPEED
        default_goal.min_rotational_speed = self.MIN_ROTATIONAL_SPEED
        default_goal.max_linear_speed = self.MAX_LINEAR_SPEED
        default_goal.max_rotational_speed = self.MAX_ROTATIONAL_SPEED
        default_goal.linear_goal_tolerance = self.LINEAR_GOAL_TOLERANCE
        default_goal.rotational_goal_tolerance = self.ROTATIONAL_GOAL_TOLERANCE
        default_goal.allowed_contact_forces.force.x = self.FORCE_LIMIT
        default_goal.allowed_contact_forces.force.y = self.FORCE_LIMIT
        default_goal.allowed_contact_forces.force.z = self.FORCE_LIMIT
        default_goal.allowed_contact_forces.torque.x = self.TORQUE_LIMIT
        default_goal.allowed_contact_forces.torque.y = self.TORQUE_LIMIT
        default_goal.allowed_contact_forces.torque.z = self.TORQUE_LIMIT
        default_goal.external_force_sensor = False
        default_goal.goal.orientation.w = 1.0
        self.__update_position(default_goal)
        return default_goal

    def send_soft_touch_goal(self, wait_for_result=True, timeout=None):
        """Send a Soft Touch goal."""
        self._client.send_soft_touch_goal(self._desired_goal)
        return self._client.wait_for_result(timeout) if wait_for_result else True

    def get_state(self):
        """.Get the action client state information of current goal.

        @return: the goal's state
        """
        return self._client.get_state()

    def wait_for_result(self, timeout=None):
        """
        Wait for an result of the commanded action.

        @param timeout: maximum time to wait.
        @return None if timeout is reached, else
                True if the goal is achieved
                False if failed to achieve goal
        """
        return self._client.wait_for_result(timeout)

    def reset_goal_configuration(self):
        """Reset the soft touch goal."""
        self._desired_goal = self.__create_default_soft_touch_goal()

    def get_current_position_as_goal(self):
        """Get the configured goal for the action with the actual position stored."""
        self.__update_position(self._desired_goal)
        return self._desired_goal

    def update_goal(self, goal):
        """
        Set a new goal for the action.

        @param goal: SoftTouchCommandGoal
        Set a new goal for the action.
        It is recommended to only adapt the values for position x,y,z and the wrench
        (torque and forces). The feature to use an external force sensor is not implemented yet.
        """
        if isinstance(goal, SoftTouchCommandGoal):
            self._desired_goal = deepcopy(goal)
        else:
            rospy.logerr('Cannot set goal data. Invalid type.')
