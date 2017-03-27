#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
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

"""
SDK Gripper Example: keyboard
"""
import argparse

import rospy

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION

def map_keyboard(limb):
    # initialize interfaces
    print("Getting robot state...")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state()
    try:
        gripper = intera_interface.Gripper(limb)
    except ValueError:
        rospy.logerr("Could not detect a gripper attached to the robot.")
        return
    def clean_shutdown():
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)
    def offset_position(offset):
        current = gripper.get_position()
        gripper.set_position(current + offset)

    def offset_holding(offset):
        current = gripper.get_force()
        gripper.set_holding_force(current + offset)

    num_steps = 10.0
    thirty_percent_velocity = 0.3*(gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) + gripper.MIN_VELOCITY
    bindings = {
    #   key: (function, args, description)
        'r': (gripper.reboot, [], "reboot"),
        'c': (gripper.calibrate, [], "calibrate"),
        'q': (gripper.close, [], "close"),
        'o': (gripper.open, [], "open"),
        '+': (gripper.set_velocity, [gripper.MAX_VELOCITY], "set 100% velocity"),
        '-': (gripper.set_velocity, [thirty_percent_velocity], "set 30% velocity"),
        's': (gripper.stop, [], "stop"),
        'h': (offset_holding, [-(gripper.MAX_FORCE / num_steps)], "decrease holding force"),
        'j': (offset_holding, [gripper.MAX_FORCE / num_steps], "increase holding force"),
        'u': (offset_position, [-(gripper.MAX_POSITION / num_steps)], "decrease position"),
        'i': (offset_position, [gripper.MAX_POSITION / num_steps], "increase position"),
    }

    done = False
    rospy.loginfo("Enabling robot...")
    rs.enable()
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = intera_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Gripper Example: Keyboard Control

    Use your dev machine's keyboard to control and configure grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, velocity,
    and force. Uses the intera_interface.Gripper class and the
    helper function, intera_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the gripper keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_gripper_keyboard")

    map_keyboard(args.limb)


if __name__ == '__main__':
    main()
