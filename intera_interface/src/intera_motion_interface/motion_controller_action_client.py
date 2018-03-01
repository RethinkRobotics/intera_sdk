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
import actionlib
import intera_motion_msgs.msg


class MotionControllerActionClient(object):
    """
    Simple ROS action client for sending trajectory messages to
    the Motion Controller.
    """

    def __init__(self):
        """
        Constructor - creates a new motion controller action client and
        waits for a connection to the motion controller action server.
        """
        self._waypointSequenceId = 0
        self._client = actionlib.SimpleActionClient(
            '/motion/motion_command',
            intera_motion_msgs.msg.MotionCommandAction)
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Timed out waiting for Motion Controller"
                         " Server to connect. Check your ROS networking"
                         "  and/or reboot the robot.")
            rospy.signal_shutdown("Timed out waiting for Action Server")

    def stop_trajectory(self):
        """
        Send a Motion Stop command
        """
        goal = intera_motion_msgs.msg.MotionCommandGoal()
        goal.command = intera_motion_msgs.msg.MotionCommandGoal.MOTION_STOP
        self._client.send_goal(goal)
        rospy.loginfo('Stopping trajectory')

    def send_trajectory(self, trajectory):
        """
        @param trajectory: a Trajectory.msg that will be packaged up with the
            MOTION_START command and sent to the motion controller.
        @return: (void) immediately after sending
        """
        goal = intera_motion_msgs.msg.MotionCommandGoal()
        goal.command = intera_motion_msgs.msg.MotionCommandGoal.MOTION_START
        goal.trajectory = trajectory
        self._client.send_goal(goal)

    def wait_for_result(self, timeout=None):
        """
        Wait for the current task to finish and then return the result
        @param timeout: maximum time to wait. If timeout is reached, return None.
        @return: the result of the MotionCommand.action
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
