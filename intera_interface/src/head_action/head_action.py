# Copyright (c) 2013-2015, Rethink Robotics
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

"""
Baxter RSDK Head Action Server
"""
from math import fabs

import rospy

import actionlib

from control_msgs.msg import (
    SingleJointPositionAction,
    SingleJointPositionFeedback,
    SingleJointPositionResult, 
)

import baxter_interface
from baxter_core_msgs.msg import (
   HeadPanCommand,
)


class HeadActionServer(object):
    def __init__(self, reconfig_server):
        self._dyn = reconfig_server
        self._ns = 'robot/head/head_action'
        self._head = baxter_interface.Head()

        # Action Server
        self._server = actionlib.SimpleActionServer(
            self._ns,
            SingleJointPositionAction,
            execute_cb=self._on_head_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()

        # Action Feedback/Result
        self._fdbk = SingleJointPositionFeedback()
        self._result = SingleJointPositionResult()

        # Initialize Parameters
        self._prm = {"dead_zone" : baxter_interface.settings.HEAD_PAN_ANGLE_TOLERANCE}
        self._timeout = 5.0

    def _get_head_parameters(self):
        self._timeout = self._dyn.config['timeout']
        self._prm['dead_zone'] = self._dyn.config['goal']

    def _update_feedback(self):
        self._fdbk.position = self._head.pan()
        self._result = self._fdbk
        self._server.publish_feedback(self._fdbk)

    def _command_head(self, position, speed):
        self._head.set_pan(position, speed, timeout=self._timeout)

    def _check_state(self, position):
        return (fabs(self._head.pan() - position) <
                self._prm['dead_zone'])

    def _on_head_action(self, goal):
        position = goal.position
        velocity = goal.max_velocity
        # Apply max velocity if specified < 0
        if velocity < HeadPanCommand.MIN_SPEED_RATIO:
            velocity = HeadPanCommand.MAX_SPEED_RATIO

        # Pull parameters that will define the head actuation
        self._get_head_parameters()

        # Reset feedback/result
        self._update_feedback()

        # 20 Hz head state rate
        control_rate = rospy.Rate(20.0)

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.get_time() - start

        # Continue commanding goal until success or timeout
        while ((now_from_start(start_time) < self._timeout or
               self._timeout < 0.0) and not rospy.is_shutdown()):
            if self._server.is_preempt_requested():
                rospy.loginfo("%s: Head Action Can't Preempted" %
                              (self._action_name,))
                self._server.set_preempted(self._result)
                return
            self._update_feedback()
            if self._check_state(position):
                self._server.set_succeeded(self._result)
                return
            self._command_head(position, velocity)
            control_rate.sleep()

        if not rospy.is_shutdown():
            rospy.logerr("%s: Head Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback()
        self._server.set_aborted(self._result)
