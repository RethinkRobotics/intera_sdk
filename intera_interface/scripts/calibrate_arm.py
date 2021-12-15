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
import sys

import rospy
import actionlib

import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import (
    CalibrationCommandAction,
    CalibrationCommandGoal,
)

class CalibrateArm(object):
    def __init__(self, limb="right"):
        """
        @param limb: Limb to run CalibrateArm on arm side.
        """
        self._limb=limb
        self._client = actionlib.SimpleActionClient('/calibration_command',
                                   CalibrationCommandAction)
        # Waits until the action server has started up and started
        # listening for goals.
        server_up = self._client.wait_for_server(rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Calibration"
                         " Server to connect. Check your ROS networking"
                         "  and/or reboot the robot.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

    def _feedback(self, data):
       """
       Prints Calibration Command Progress
       """
       ratio=float(data.currentPoseNumber)/float(data.numberOfPoses)
       print('[{0}] {1}% complete - {2}'.format(
               ('#'*int(ratio*40)).ljust(40),
               '{0:.2f}'.format(ratio*100.0).rjust(5, ' '),
               data.currentState.ljust(10)),
               end='\r')

    def _send_calibration_msg(self, cmd):
        """
        Sends Calibration Command request
        """
       # Creates a goal to send to the action server.
        msg = CalibrationCommandGoal()
        msg.command = cmd
        # Sends the goal to the action server.
        rospy.loginfo("Sending Calibration Request {0}.".format(
                        cmd.capitalize()))
        self._client.send_goal(msg, feedback_cb=self._feedback)
        # Waits for the server to finish performing the action.
        self._client.wait_for_result()
        # Prints out the result of executing the action
        return self._client.get_result()

    def stop_calibration(self):
        """
        Preempts calibration execution by sending cancel goal
        """
        return self._send_calibration_msg(CalibrationCommandGoal.CALIBRATION_STOP)

    def start_calibration(self):
        """
        Starts calibration execution by sending goal
        """
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        # Enable robot
        rs.enable()
        l = intera_interface.Limb(self._limb)
        # Move slowly to set neutral pose
        rospy.loginfo("Moving {0} arm to neutral pose...".format(self._limb))
        l.move_to_neutral(timeout=25.0, speed=0.1)
        # Ask robot to calibrate
        cal_result = self._send_calibration_msg(CalibrationCommandGoal.CALIBRATION_START)
        l.set_joint_position_speed(speed=0.3)
        return cal_result

def is_gripper_removed():
    """
    Verify grippers are removed for calibration.
    """
    try:
        gripper = intera_interface.get_current_gripper_interface()
    except Exception as e:
        return True
    rospy.logerr("Calibration Client: Cannot calibrate with grippers attached."
                 " Remove grippers before calibration!")
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb',
                        choices=['left', 'right'], default="right",
                        help="Calibrate the specified arm")
    args = parser.parse_args(rospy.myargv()[1:])
    arm = args.limb

    print("Initializing node...")
    rospy.init_node('sdk_calibrate_arm_{0}'.format(arm), disable_signals=True)

    rospy.loginfo("Preparing to Calibrate...")
    gripper_warn = ("IMPORTANT: Make sure to remove grippers and other"
                    " attachments before running Calibrate.")
    rospy.loginfo(gripper_warn)
    if not is_gripper_removed():
        return 1

    ca = CalibrateArm(arm)

    error = None
    goal_state = "unreported error"
    rospy.loginfo("Running Calibrate on {0} arm".format(arm))
    try:
        goal_state = ca.start_calibration()
    except KeyboardInterrupt as e:
        error = e
        goal_state = ca.stop_calibration()

    if error == None and "success" in str(goal_state).lower():
        rospy.loginfo("Calibrate arm finished successfully. Please reboot your robot to use this calibration data.")
    else:
        error = True
        rospy.logerr("Calibrate arm failed with {0}".format(goal_state))
        rospy.logerr("Please re-run this Calibration request.")

    return 0 if error == None else 1

if __name__ == '__main__':
    sys.exit(main())
