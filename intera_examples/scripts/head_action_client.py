#!/usr/bin/env python

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
Baxter RSDK Head Action Client Example
"""
import sys
import argparse

import rospy

import actionlib

from control_msgs.msg import (
    SingleJointPositionAction,
    SingleJointPositionGoal,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

class HeadClient(object):
    def __init__(self):
        ns = 'robot/head/head_action'
        self._client = actionlib.SimpleActionClient(
            ns,
            SingleJointPositionAction
        )
        self._goal = SingleJointPositionGoal()

        # Wait 10 Seconds for the head action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Head Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

    def command(self, position, velocity):
        self._goal.position = position
        self._goal.max_velocity = velocity
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=5.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = SingleJointPositionGoal()

def main():
    """RSDK Head Example: Action Client

    Demonstrates creating a client of the Head Action Server,
    which enables sending commands of standard action type
    control_msgs/SingleJointPosition.

    The example will command the head to a position.
    Be sure to start Baxter's head_action_server before running this example.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_head_action_client")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    hc = HeadClient()
    hc.command(position=0.0, velocity=1.0)
    hc.wait()
    hc.command(position=1.57, velocity=0.1)
    hc.wait()
    hc.command(position=0.0, velocity=0.8)
    hc.wait()
    hc.command(position=-1.0, velocity=0.4)
    hc.wait()
    hc.command(position=0.0, velocity=0.6)
    print hc.wait()
    print "Exiting - Head Action Test Example Complete"

if __name__ == "__main__":
    main()
