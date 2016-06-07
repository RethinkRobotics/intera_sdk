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
Baxter RSDK Gripper Example: joystick
"""
import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def map_joystick(joystick):
    """
    maps joystick input to gripper commands

    @param joystick: an instance of a Joystick
    """
    # initialize interfaces
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    right = baxter_interface.Gripper('right', CHECK_VERSION)

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    # decrease position dead_band
    left.set_dead_band(2.5)
    right.set_dead_band(2.5)

    # abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1]), doc))

    def capability_warning(gripper, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (gripper.name, gripper.type(), cmd))
        print msg

    def offset_position(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_position')
            return
        current = gripper.position()
        gripper.command_position(current + offset)

    def offset_holding(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_holding_force')
            return
        current = gripper.parameters()['holding_force']
        gripper.set_holding_force(current + offset)

    def offset_velocity(gripper, offset):
        if gripper.type() != 'electric':
            capability_warning(gripper, 'set_velocity')
            return
        current = gripper.parameters()['velocity']
        gripper.set_velocity(current + offset)

    bindings_list = []
    bindings = (
        #(test, command, description)
        ((bdn, ['btnDown']), (left.reboot, []), "left: reboot"),
        ((bdn, ['btnLeft']), (right.reboot, []), "right: reboot"),
        ((bdn, ['btnRight']), (left.calibrate, []), "left: calibrate"),
        ((bdn, ['btnUp']), (right.calibrate, []), "right: calibrate"),
        ((bdn, ['rightTrigger']), (left.close, []), "left: close"),
        ((bdn, ['leftTrigger']), (right.close, []), "right: close"),
        ((bup, ['rightTrigger']), (left.open, []), "left: open (release)"),
        ((bup, ['leftTrigger']), (right.open, []), "right: open (release)"),
        ((bdn, ['rightBumper']), (left.stop, []), "left: stop"),
        ((bdn, ['leftBumper']), (right.stop, []), "right: stop"),
        ((jlo, ['rightStickHorz']), (offset_position, [left, -15.0]),
                                     "left:  decrease position"),
        ((jlo, ['leftStickHorz']), (offset_position, [right, -15.0]),
                                    "right:  decrease position"),
        ((jhi, ['rightStickHorz']), (offset_position, [left, 15.0]),
                                     "left:  increase position"),
        ((jhi, ['leftStickHorz']), (offset_position, [right, 15.0]),
                                     "right:  increase position"),
        ((jlo, ['rightStickVert']), (offset_holding, [left, -5.0]),
                                     "left:  decrease holding force"),
        ((jlo, ['leftStickVert']), (offset_holding, [right, -5.0]),
                                    "right:  decrease holding force"),
        ((jhi, ['rightStickVert']), (offset_holding, [left, 5.0]),
                                     "left:  increase holding force"),
        ((jhi, ['leftStickVert']), (offset_holding, [right, 5.0]),
                                    "right:  increase holding force"),
        ((bdn, ['dPadDown']), (offset_velocity, [left, -5.0]),
                               "left:  decrease velocity"),
        ((bdn, ['dPadLeft']), (offset_velocity, [right, -5.0]),
                               "right:  decrease velocity"),
        ((bdn, ['dPadRight']), (offset_velocity, [left, 5.0]),
                                "left:  increase velocity"),
        ((bdn, ['dPadUp']), (offset_velocity, [right, 5.0]),
                             "right:  increase velocity"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

    print("Enabling robot...")
    rs.enable()
    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press <Start> button for help; Ctrl-C to stop...")
    while not rospy.is_shutdown():
        # test each joystick condition and call binding cmd if true
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                print(doc)
        rate.sleep()
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Gripper Example: Joystick Control

    Use a game controller to control the grippers.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control Baxter's
    grippers using the joysticks and buttons. Be sure to provide
    the *joystick* type you are using as an argument to setup
    appropriate key mappings.

    Uses the baxter_interface.Gripper class and the helper classes
    in baxter_external_devices.Joystick.
    """
    epilog = """
See help inside the example with the "Start" button for controller
key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True, choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = baxter_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_external_devices.joystick.PS3Controller()
    else:
        # Should never reach this case with proper argparse usage
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_joystick")

    map_joystick(joystick)


if __name__ == '__main__':
    main()
