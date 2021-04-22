#! /usr/bin/env python
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

import argparse
import copy
import threading

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import RobotAssemblyState, HomingState, HomingCommand

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
            RobotAssemblyState,
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
            self._homing_state = dict(list(zip(msg.name, msg.state)))

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
                    homing_joints = copy.deepcopy(list(self._homing_state.keys()))
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
