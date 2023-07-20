#! /usr/bin/env python
# ****************************************************************************
# softtouch_action_client.py
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

import rospy
import actionlib
from intera_motion_msgs.msg import SoftTouchCommandAction


class SoftTouchActionClient(object):
    """Simple ROS action client for sending Soft Touch Commands to the SoftTouchActionServer."""

    def __init__(self):
        """
        Construct a new SoftTouchClient.

        Create a new client and wait for a connection to the Soft Touch action server.
        """
        self._client = actionlib.SimpleActionClient('soft_touch_skill', SoftTouchCommandAction)
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr(
                """
                Timed out waiting for SoftTouch Controller
                Server to connect. Check your ROS networking'
                and/or reboot the robot.
                """
            )
            rospy.signal_shutdown('Timed out waiting for Action Server')

    def send_soft_touch_goal(self, soft_touch_goal):
        """
        Send goal to Action Server.

        @param soft_touch_goal: a goal for the Soft Touch action which will be sent to the
        action server.
        @return: (void) immediately after sending
        """
        goal = soft_touch_goal
        self._client.send_goal(goal)

    def wait_for_result(self, timeout=None):
        """
        Wait for the current task to finish and then return the result.

        @param timeout: maximum time to wait. If timeout is reached, return None.
        @return: the result of the SoftTouchCommand.action
        """
        if timeout is None:
            self._client.wait_for_result()
        else:
            self._client.wait_for_result(rospy.Duration(timeout))
        return self._client.get_result()

    def get_state(self):
        """
        Get the state information of current goal.

        @return: the goal's state
        """
        return self._client.get_state()
