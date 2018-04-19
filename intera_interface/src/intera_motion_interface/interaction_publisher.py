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
from interaction_options import InteractionOptions
import intera_interface
from intera_interface import CHECK_VERSION

class InteractionPublisher(object):
    """
    Send Interaction Commands to robot
    """

    def __init__(self):
        self.pub = rospy.Publisher('/robot/limb/right/interaction_control_command',
                                  InteractionControlCommand, queue_size = 1)
        rospy.sleep(0.5)

    def send_command(self, msg, pub_rate):
        repeat = False
        if pub_rate > 0:
            rate = rospy.Rate(pub_rate)
            repeat = True
        elif pub_rate == 0:
            rospy.logwarn('Interaction control options will be set only once!')
        elif pub_rate < 0:
            rospy.logerr('Invalid publish rate!')

        try:
            # print the resultant interaction options once
            rospy.loginfo(msg)
            self.pub.publish(msg)
            rs = intera_interface.RobotEnable(CHECK_VERSION)
            while repeat and not rospy.is_shutdown() and rs.state().enabled:
                rate.sleep()
                self.pub.publish(msg)
        except rospy.ROSInterruptException:
            rospy.logerr('Keyboard interrupt detected from the user. %s',
                         'Exiting the node...')
        if not rs.state().enabled:
            self.send_position_mode_cmd()

    def send_position_mode_cmd(self):
        # send a message to put the robot back into position mode
        position_mode = InteractionOptions()
        position_mode.set_interaction_control_active(False)
        self.pub.publish(position_mode.to_msg())
        rospy.loginfo('Sending position command')
        rospy.sleep(0.5)
