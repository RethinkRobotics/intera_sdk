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
    Interface class for the head on an Intera Robot.

    Used to control the head pan angle.
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

        self._tf_listener = tf.TransformListener()

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

        @rtype: string
        @return: current mode -
                 'PASSIVE_MODE'(0) : Compliant to user-induced external movement
                 'ACTIVE_MODE' (1)  : Actively responds to absolute commanded
                                    position
                                    Command limits are actual joint limits.
                 'ACTIVE_CANCELLATION_MODE' (2) : Actively responds to commanded
                                           head position relative to the
                                           current position of robot base frame
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
                        the "angle" param is measured with respect to
                        the "/base" frame, rather than the actual head joint
                        value. Valid range is [-pi, pi) radians.
        @type active_cancellation: bool
        """
        if speed > HeadPanCommand.MAX_SPEED_RATIO:
            rospy.logwarn(("Commanded Speed, ({0}), faster than Max speed of"
                          " {1}. Clamping to Max.]").format(speed,
                          HeadPanCommand.MAX_SPEED_RATIO))
            speed = HeadPanCommand.MAX_SPEED_RATIO
        elif speed < HeadPanCommand.MIN_SPEED_RATIO:
            rospy.logwarn(("Commanded Speed, ({0}), slower than Min speed of"
                           " {1}. Clamping to Min.]").format(speed,
                           HeadPanCommand.MIN_SPEED_RATIO))
            speed = HeadPanCommand.MIN_SPEED_RATIO
        if active_cancellation:
            def get_current_euler(axis, source_frame="base",
                                  target_frame="head"):
                rate = rospy.Rate(10) # Hz
                counter = 1
                quat = (0,0,0,1)
                while not rospy.is_shutdown():
                    try:
                        pos, quat = self._tf_listener.lookupTransform(
                            source_frame, target_frame,
                            self._tf_listener.getLatestCommonTime(source_frame,
                            target_frame))
                    except tf.Exception:
                        if not counter % 10: # essentially throttle 1.0 sec
                            rospy.logwarn(("Active Cancellation: Trying again "
                                           "to lookup transform from {0} to "
                                           "{1}...").format(source_frame,
                                           target_frame))
                    else:
                        break
                    counter += 1
                    rate.sleep()
                euler = tf.transformations.euler_from_quaternion(quat)
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
