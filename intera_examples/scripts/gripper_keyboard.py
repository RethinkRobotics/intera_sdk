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
    gripper = None
    original_deadzone = None
    def clean_shutdown():
        if gripper and original_deadzone:
            gripper.set_dead_zone(original_deadzone)
        print("Exiting example.")
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except (ValueError, OSError) as e:
        rospy.logerr("Could not detect an electric gripper attached to the robot.")
        clean_shutdown()
        return
    rospy.on_shutdown(clean_shutdown)

    def offset_position(offset_pos):
        cmd_pos = max(min(gripper.get_position() + offset_pos, gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))

    def update_velocity(offset_vel):
        cmd_speed = max(min(gripper.get_cmd_velocity() + offset_vel, gripper.MAX_VELOCITY), gripper.MIN_VELOCITY)
        gripper.set_cmd_velocity(cmd_speed)
        print("commanded velocity set to {0} m/s".format(cmd_speed))


    original_deadzone = gripper.get_dead_zone()
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    num_steps = 8.0
    percent_delta = 1.0 / num_steps
    velocity_increment = (gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) * percent_delta
    position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta
    bindings = {
    #   key: (function, args, description)
        'r': (gripper.reboot, [], "reboot"),
        'c': (gripper.calibrate, [], "calibrate"),
        'q': (gripper.close, [], "close"),
        'o': (gripper.open, [], "open"),
        '+': (update_velocity, [velocity_increment], "increase velocity by {0}%".format(percent_delta*100)),
        '-': (update_velocity, [-velocity_increment],"decrease velocity by {0}%".format(percent_delta*100)),
        's': (gripper.stop, [], "stop"),
        'u': (offset_position, [-position_increment], "decrease position by {0}%".format(percent_delta*100)),
        'i': (offset_position, [position_increment], "increase position by {0}%".format(percent_delta*100)),
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
                print("command: {0}".format(cmd[2]))
                cmd[0](*cmd[1])
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(list(bindings.items()),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Gripper Example: Keyboard Control

    Use your dev machine's keyboard to control and configure grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, and velocity:
    Uses the intera_interface.Gripper class and the
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
