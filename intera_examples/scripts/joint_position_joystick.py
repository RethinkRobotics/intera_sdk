#!/usr/bin/env python

# Copyright (c) 2013-2016, Rethink Robotics
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
Sawyer RSDK Joint Position Example: joystick
"""
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def rotate(l):
    """
    Rotates a list left.

    @param l: the list
    """
    if len(l):
        v = l[0]
        l[:-1] = l[1:]
        l[-1] = v


def set_j(cmd, limb, joints, index, delta):
    """
    Set the selected joint to current pos + delta.

    @param cmd: the joint command dictionary
    @param limb: the limb to get the pos from
    @param joints: a list of joint names
    @param index: the index in the list of names
    @param delta: delta to update the joint by

    joint/index is to make this work in the bindings.
    """
    joint = joints[index]
    cmd[joint] = delta + limb.joint_angle(joint)


def map_joystick(joystick):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """
    right = intera_interface.Limb('right')
    ##############################################################
    # TODO: Fix gripper.py then enable gripper joystick control. #
    ##############################################################
    #grip_right = intera_interface.Gripper('right', CHECK_VERSION)
    rcmd = {}

    #available joints
    rj = right.joint_names()

    #abbreviations
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
                print("%s: %s" % (str(test[1][0]), doc))

    bindings_list = []
    bindings = (
        ##############################################################
        # TODO: Fix gripper.py then enable gripper joystick control. #
        ##############################################################
        #((bdn, ['leftTrigger']),
        # (grip_right.close, []), "right gripper close"),
        #((bup, ['leftTrigger']),
        # (grip_right.open,  []), "right gripper open"),
        ((jlo, ['leftStickHorz']),
         (set_j, [rcmd, right, rj, 0,  0.1]), lambda: "right inc " + rj[0]),
        ((jhi, ['leftStickHorz']),
         (set_j, [rcmd, right, rj, 0, -0.1]), lambda: "right dec " + rj[0]),
        ((jlo, ['leftStickVert']),
         (set_j, [rcmd, right, rj, 1,  0.1]), lambda: "right inc " + rj[1]),
        ((jhi, ['leftStickVert']),
         (set_j, [rcmd, right, rj, 1, -0.1]), lambda: "right dec " + rj[1]),
        ((bdn, ['leftBumper']),
         (rotate, [rj]), "right: cycle joint"),
        ##############################################################
        # TODO: Fix gripper.py then enable gripper joystick control. #
        ##############################################################
        #((bdn, ['btnLeft']),
        # (grip_right.calibrate, []), "right calibrate"),
        ((bdn, ['function1']),
         (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
         (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press Ctrl-C to stop. ")
    while not rospy.is_shutdown():
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        if len(rcmd):
            right.set_joint_positions(rcmd)
            rcmd.clear()
        rate.sleep()
    return False


def main():
    """RSDK Joint Position Example: Joystick Control

    Use a game controller to control the angular joint positions
    of Sawyer's arms.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Sawyer's arm using the joystick. Be sure to
    provide your *joystick* type to setup appropriate key mappings.

    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
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
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
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
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_joystick")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_joystick(joystick)
    print("Done.")


if __name__ == '__main__':
    main()
