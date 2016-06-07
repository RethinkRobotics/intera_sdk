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
Baxter RSDK Gripper Example: keyboard
"""
import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def map_keyboard():
    # initialize interfaces
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    right = baxter_interface.Gripper('right', CHECK_VERSION)

    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)

    def capability_warning(gripper, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (gripper.name, gripper.type(), cmd))
        rospy.logwarn(msg)

    def offset_position(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'command_position')
            return
        current = gripper.position()
        gripper.command_position(current + offset)

    def offset_holding(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_holding_force')
            return
        current = gripper.parameters()['holding_force']
        gripper.set_holding_force(current + offset)

    def offset_moving(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_moving_force')
            return
        current = gripper.parameters()['moving_force']
        gripper.set_moving_force(current + offset)

    def offset_velocity(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_velocity')
            return
        current = gripper.parameters()['velocity']
        gripper.set_velocity(current + offset)

    def offset_dead_band(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_dead_band')
            return
        current = gripper.parameters()['dead_zone']
        gripper.set_dead_band(current + offset)

    bindings = {
    #   key: (function, args, description)
        'r': (left.reboot, [], "left: reboot"),
        'R': (right.reboot, [], "right: reboot"),
        'c': (left.calibrate, [], "left: calibrate"),
        'C': (right.calibrate, [], "right: calibrate"),
        'q': (left.close, [], "left: close"),
        'Q': (right.close, [], "right: close"),
        'w': (left.open, [], "left: open"),
        'W': (right.open, [], "right: open"),
        '[': (left.set_velocity, [100.0], "left:  set 100% velocity"),
        '{': (right.set_velocity, [100.0], "right:  set 100% velocity"),
        ']': (left.set_velocity, [30.0], "left:  set 30% velocity"),
        '}': (right.set_velocity, [30.0], "right:  set 30% velocity"),
        's': (left.stop, [], "left: stop"),
        'S': (right.stop, [], "right: stop"),
        'z': (offset_dead_band, [left, -1.0], "left:  decrease dead band"),
        'Z': (offset_dead_band, [right, -1.0], "right:  decrease dead band"),
        'x': (offset_dead_band, [left, 1.0], "left:  increase dead band"),
        'X': (offset_dead_band, [right, 1.0], "right:  increase dead band"),
        'f': (offset_moving, [left, -5.0], "left:  decrease moving force"),
        'F': (offset_moving, [right, -5.0], "right:  decrease moving force"),
        'g': (offset_moving, [left, 5.0], "left:  increase moving force"),
        'G': (offset_moving, [right, 5.0], "right:  increase moving force"),
        'h': (offset_holding, [left, -5.0], "left:  decrease holding force"),
        'H': (offset_holding, [right, -5.0], "right:  decrease holding force"),
        'j': (offset_holding, [left, 5.0], "left:  increase holding force"),
        'J': (offset_holding, [right, 5.0], "right:  increase holding force"),
        'v': (offset_velocity, [left, -5.0], "left:  decrease velocity"),
        'V': (offset_velocity, [right, -5.0], "right:  decrease velocity"),
        'b': (offset_velocity, [left, 5.0], "left:  increase velocity"),
        'B': (offset_velocity, [right, 5.0], "right:  increase velocity"),
        'u': (offset_position, [left, -15.0], "left:  decrease position"),
        'U': (offset_position, [right, -15.0], "right:  decrease position"),
        'i': (offset_position, [left, 15.0], "left:  increase position"),
        'I': (offset_position, [right, 15.0], "right:  increase position"),
    }

    done = False
    print("Enabling robot... ")
    rs.enable()
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
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

    Use your dev machine's keyboard to control and configure
    Baxter's grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, velocity,
    and force. Uses the baxter_interface.Gripper class and the
    helper function, baxter_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_keyboard")

    map_keyboard()


if __name__ == '__main__':
    main()
