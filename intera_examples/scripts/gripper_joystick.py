#!/usr/bin/env python

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

"""
RSDK Gripper Example: joystick
"""
import argparse

import rospy

import intera_interface
import intera_external_devices


def map_joystick(joystick):
    """
    maps joystick input to gripper commands

    @param joystick: an instance of a Joystick
    """
    gripper = intera_interface.Gripper()
    # decrease position dead_band
    gripper.set_dead_zone(2.5)

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

    def capability_warning(cmd):
        msg = ("%s - not capable of '%s' command" %
               (gripper.name, cmd))
        rospy.logwarn(msg)

    def offset_position(offset):
        if gripper.name != 'right_gripper':
            capability_warning('command_position')
            return
        current = gripper.get_position()
        gripper.set_position(current + offset)

    def offset_holding(offset):
        if gripper.name != 'right_gripper':
            capability_warning('set_holding_force')
            return
        current = gripper.get_force()
        gripper.set_holding_force(current + offset)

    bindings_list = []
    bindings = (
        #(test, command, description)
        ((bdn, ['btnLeft']), (gripper.reboot, []), "reboot"),
        ((bdn, ['btnUp']), (gripper.calibrate, []), "calibrate"),
        ((bdn, ['leftTrigger']), (gripper.close, []), "close"),
        ((bup, ['leftTrigger']), (gripper.open, []), "open (release)"),
        ((bdn, ['leftBumper']), (gripper.stop, []), "stop"),
        ((jlo, ['leftStickHorz']), (offset_position, [-15.0]),
                                    "decrease position"),
        ((jhi, ['leftStickHorz']), (offset_position, [15.0]),
                                     "increase position"),
        ((jlo, ['leftStickVert']), (offset_holding, [-5.0]),
                                    "decrease holding force"),
        ((jhi, ['leftStickVert']), (offset_holding, [5.0]),
                                    "increase holding force"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

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
    example along with the ROS joy_node to control gripper
    using the joysticks and buttons. Be sure to provide
    the *joystick* type you are using as an argument to setup
    appropriate key mappings.

    Uses the intera_interface.Gripper class and the helper classes
    in intera_external_devices.Joystick.
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
        joystick = intera_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = intera_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = intera_external_devices.joystick.PS3Controller()
    else:
        # Should never reach this case with proper argparse usage
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_gripper_joystick")

    map_joystick(joystick)


if __name__ == '__main__':
    main()