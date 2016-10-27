#!/usr/bin/python

# Copyright (c) 2016, Rethink Robotics
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

import argparse
import copy
import threading

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import AssemblyState, HomingState, HomingCommand

class HomeJoints(object):
    def __init__(self):
        """
        HomeJoints - Class that publishes on /robot/set_homing_mode to
        home the robot if it is not already homed.
        """
        self._hcb_lock = threading.Lock()
        self._ecb_lock = threading.Lock()
        self._homing_state = dict()
        self._enable_state = False
        self._pub_home_joints = rospy.Publisher(
            '/robot/set_homing_mode',
            HomingCommand,
            latch=True,
            queue_size=10)
        self._enable_sub = rospy.Subscriber(
            '/robot/state',
            AssemblyState,
            self._enable_state_cb)
        self._homing_sub = rospy.Subscriber(
            '/robot/homing_states',
            HomingState,
            self._homing_state_cb)

    def _enable_state_cb(self, msg):
        with self._ecb_lock:
            self._enable_state =  msg.enabled

    def _robot_is_disabled(self):
        with self._ecb_lock:
            return False if self._enable_state else True

    def _homing_state_cb(self, msg):
        with self._hcb_lock:
            self._homing_state = dict(zip(msg.name, msg.state))

    def _joints_are_homed(self):
        with self._hcb_lock:
            if not self._homing_state:
                return False
            for joint_name in self._homing_state:
                if self._homing_state[joint_name] != HomingState.HOMED:
                    return False
            return True

    def home_robot(self, mode=HomingCommand.AUTO, timeout=60.0):
        sleep_rate = rospy.Rate(1)
        while not rospy.is_shutdown() and self._robot_is_disabled():
            sleep_rate.sleep()
        start_time = rospy.Time.now()
        timeout_reached = lambda:((rospy.Time.now() - start_time).to_sec() > timeout)
        # Wait for First Homing State Callback
        homing_joints = list()
        while not rospy.is_shutdown():
            if timeout_reached():
                return False
            with self._hcb_lock:
                if bool(self._homing_state):
                    homing_joints = copy.deepcopy(self._homing_state.keys())
                    break
            sleep_rate.sleep()
        # Construct Homing Command
        cmd = HomingCommand()
        cmd.name = homing_joints
        cmd.command = [mode]*len(homing_joints)
        # Publish & Wait for Joints to Home
        while not rospy.is_shutdown():
            if timeout_reached():
                break
            if not self._joints_are_homed():
                # Publish Homing command
                self._pub_home_joints.publish(cmd)
            else:
                return True
            sleep_rate.sleep()
        return False

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=lambda t:abs(float(t)),
            default=60.0, help="[in seconds] Time to wait for joints to home")
    parser.add_argument("-m", "--mode", type=str.upper, default="AUTO",
            choices=['AUTO', 'MANUAL'], help="Mode to home the robot's joints")
    enable_parser = parser.add_mutually_exclusive_group(required=False)
    enable_parser.add_argument("-e", "--enable", action='store_true', dest='enable',
                       help="Try to enable the robot before homing.")
    enable_parser.add_argument("-n", "--no-enable", action='store_false', dest='enable',
                       help="Avoid trying to enable the robot before homing.")
    enable_parser.set_defaults(enable=True)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('home_joints_node')
    if args.enable:
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
    cmd_mode = HomingCommand.MANUAL if args.mode == 'MANUAL' else HomingCommand.AUTO
    rospy.loginfo("Homing joints in '{0}' mode".format(args.mode.capitalize()))
    home_jnts = HomeJoints()
    state = home_jnts.home_robot(mode=cmd_mode, timeout=args.timeout)
    rospy.loginfo("{0} in homing the robot's joints".format("Succeeded" if state else "Failed"))

if __name__ == '__main__':
    main()
