# Copyright (c) 2013-2016, Rethink Robotics
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

from copy import deepcopy
from math import fabs, pi

import rospy
import tf

from std_msgs.msg import (
    Bool
)

import intera_dataflow

from intera_core_msgs.msg import (
   HeadPanCommand,
   HeadState,
)
from intera_interface import settings


class Head(object):
    """
    Interface class for the head on the Baxter Robot.

    Used to control the head pan angle and to enable/disable the head nod
    action.
    """
    def __init__(self):
        """
        Constructor.
        """
        self._state = dict()

        self._pub_pan = rospy.Publisher(
            '/robot/head/command_head_pan',
            HeadPanCommand,
            queue_size=10)

        state_topic = '/robot/head/head_state'
        self._sub_state = rospy.Subscriber(
            state_topic,
            HeadState,
            self._on_head_state)

        intera_dataflow.wait_for(
            lambda: len(self._state) != 0,
            timeout=5.0,
            timeout_msg=("Failed to get current head state from %s" %
                         (state_topic,)),
        )

    def _on_head_state(self, msg):
        self._state['pan'] = msg.pan
        self._state['panning'] = msg.isTurning
        self._state['blocked'] = msg.isBlocked
        self._state['pan_mode'] = msg.panMode

    def blocked(self):
        """
        Check if the head is currently blocked from movement.
        Get the current pan angle of the head. This can only
        be true if 'pan_mode' is ACTIVE_CANCELLATION_MODE.

        @rtype: bool
        @return: True if the head is currently blocked, False otherwise.
        """
        return self._state['blocked']

    def pan_mode(self):
        """
        Get the mode the head is currently acting in.

        @rtype: uint8
        @return: current mode -
                 'PASSIVE_MODE'(0) : Compliant to user-induced external movement
                 'ACTIVE_MODE' (1)  : Actively responds to absolute commanded
                                    position
                                    Command limits are actual joint limits.
                 'ACTIVE_CANCELLATION_MODE' (2) : Actively responds to commanded
                                           head position relative to the
                                           current position of j0 joint
                                           Command limits are [-pi, pi] rads.
        """
        pan_mode_dict = {0:'PASSIVE_MODE', 1:'ACTIVE_MODE',
                         2:'ACTIVE_CANCELLATION_MODE'}
        return pan_mode_dict[self._state['pan_mode']]

    def pan(self):
        """
        Get the current pan angle of the head.

        @rtype: float
        @return: current angle in radians
        """
        return self._state['pan']

    def panning(self):
        """
        Check if the head is currently panning.

        @rtype: bool
        @return: True if the head is currently panning, False otherwise.
        """
        return self._state['panning']

    def set_pan(self, angle, speed=1.0, timeout=10.0,
                active_cancellation=False):
        """
        Pan at the given speed to the desired angle.

        @type angle: float
        @param angle: Desired pan angle in radians.
        @type speed: int
        @param speed: Desired speed to pan at, range is 0-1.0 [1.0]
        @type timeout: float
        @param timeout: Seconds to wait for the head to pan to the
                        specified angle. If 0, just command once and
                        return. [10]
        @param active_cancellation: Specifies if the head should aim at
                        a location in the base frame. If this is set to True,
                        the "angle" argument is measured from Zero degrees in
                        the "/base" frame, rather than the actual head joint
                        value. Valid range is [-pi, pi)
        @type active_cancellation: bool
        """
        if (speed < HeadPanCommand.MIN_SPEED_RATIO or
            speed > HeadPanCommand.MAX_SPEED_RATIO):
            rospy.logerr(("Commanded Speed, ({0}), outside of valid range"
                          " [{1}, {2}]").format(speed,
                          HeadPanCommand.MIN_SPEED_RATIO,
                          HeadPanCommand.MAX_SPEED_RATIO))
        if active_cancellation:
            listener = tf.TransformListener()
            def get_current_euler(axis, source_frame="base",
                                  target_frame="head"):
                rate = rospy.Rate(10) # Hz
                counter = 1
                quaternion = (0,0,0,1)
                while not rospy.is_shutdown():
                    try:
                        position, quaternion = listener.lookupTransform(
                            source_frame, target_frame,
                            listener.getLatestCommonTime(source_frame,
                            target_frame))
                    except tf.Exception:
                        if not counter % 10: # essentially throttle 1.0 sec
                            rospy.logwarn(("Active Cancellation: Trying again "
                                           "to lookup transform from {0} to "
                                           "{1}...").format(source_frame,
                                           target_frame))
                    else:
                        break;
                    counter += 1;
                    rate.sleep()
                euler = tf.transformations.euler_from_quaternion(quaternion)
                return euler[axis]
            tf_angle = -pi + (angle + pi) % (2*pi)
            mode = HeadPanCommand.SET_ACTIVE_CANCELLATION_MODE
            stop_condition = lambda: (abs(get_current_euler(2) - tf_angle) <=
                               settings.HEAD_PAN_ANGLE_TOLERANCE)
        else:
            mode = HeadPanCommand.SET_ACTIVE_MODE
            stop_condition = lambda: (abs(self.pan() - angle) <=
                               settings.HEAD_PAN_ANGLE_TOLERANCE)
        msg = HeadPanCommand(angle, speed, mode)
        self._pub_pan.publish(msg)
        if not timeout == 0:
            intera_dataflow.wait_for(
                stop_condition,
                timeout=timeout,
                rate=100,
                timeout_msg=("Failed to move head to pan"
                             " command {0}").format(angle),
                body=lambda: self._pub_pan.publish(msg)
                )
