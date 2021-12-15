#! /usr/bin/env python
# Copyright (c) 2018, Rethink Robotics Inc.
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
from intera_core_msgs.msg import InteractionControlCommand
from .interaction_options import InteractionOptions
import intera_interface
from intera_interface import CHECK_VERSION

class InteractionPublisher(object):
    """
    ROS publisher for sending command messages to robot
    """

    def __init__(self):
        """
        Constructor - creates a publisher for interaction_control_command
        Note, that the program may need to sleep for 0.5 seconds before the
        publisher is established.
        """
        self.pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                   InteractionControlCommand, queue_size=1,
                                   tcp_nodelay=True)
        self.enable = intera_interface.RobotEnable(CHECK_VERSION)

    def send_command(self, msg, pub_rate):
        """
        @param msg: either an InteractionControlCommand message or
                    InteractionOptions object to be published
        @param pub_rate: the rate in Hz to publish the command

        Note that this function is blocking for non-zero pub_rates until
        the node is shutdown (e.g. via cntl+c) or the robot is disabled.
        A pub_rate of zero will publish the function once and return.
        """
        repeat = False
        if pub_rate > 0:
            rate = rospy.Rate(pub_rate)
            repeat = True
        elif pub_rate < 0:
            rospy.logerr('Invalid publish rate!')

        if isinstance(msg, InteractionOptions):
            msg = msg.to_msg()

        try:
            self.pub.publish(msg)
            while repeat and not rospy.is_shutdown() and self.enable.state().enabled:
                rate.sleep()
                self.pub.publish(msg)
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. %s',
                         'Exiting the node...')
        finally:
            if repeat:
                self.send_position_mode_cmd()

    def send_position_mode_cmd(self):
        '''
        Send a message to put the robot back into position mode
        '''
        position_mode = InteractionOptions()
        position_mode.set_interaction_control_active(False)
        self.pub.publish(position_mode.to_msg())
        rospy.loginfo('Sending position command')
        rospy.sleep(0.5)
