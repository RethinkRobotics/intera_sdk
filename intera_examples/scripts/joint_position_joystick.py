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
SDK Joint Position Example: joystick
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

def map_joystick(joystick, side):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """
    limb = intera_interface.Limb(side)
    gripper = None
    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        rospy.loginfo("Could not detect a connected electric gripper.")

    def set_g(action):
        if gripper is not None:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    limb_cmd = {}

    #available joints
    joints = limb.joint_names()

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
    bindings = [
        ((jlo, ['leftStickHorz']), (set_j, [limb_cmd, limb, joints, 0, 0.1]),
            lambda: "Increase " + joints[0]),
        ((jhi, ['leftStickHorz']), (set_j, [limb_cmd, limb, joints, 0, -0.1]),
            lambda: "Decrease " + joints[0]),
        ((jlo, ['leftStickVert']), (set_j, [limb_cmd, limb, joints, 1, 0.1]),
            lambda: "Increase " + joints[1]),
        ((jhi, ['leftStickVert']), (set_j, [limb_cmd, limb, joints, 1, -0.1]),
            lambda: "Decrease " + joints[1]),
        ((jlo, ['rightStickHorz']), (set_j, [limb_cmd, limb, joints, 2, 0.1]),
            lambda: "Increase " + joints[2]),
        ((jhi, ['rightStickHorz']), (set_j, [limb_cmd, limb, joints, 2, -0.1]),
            lambda: "Decrease " + joints[2]),
        ((jlo, ['rightStickVert']), (set_j, [limb_cmd, limb, joints, 3, 0.1]),
            lambda: "Increase " + joints[3]),
        ((jhi, ['rightStickVert']), (set_j, [limb_cmd, limb, joints, 3, -0.1]),
            lambda: "Decrease " + joints[3]),
        ((bdn, ['leftBumper']), (rotate, [joints]), side + ": cycle joint"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
        ]
    if gripper:
        bindings.extend([
            ((bdn, ['rightTrigger']), (set_g, ['close'], gripper), side + " gripper close"),
            ((bup, ['rightTrigger']), (set_g, ['open'], gripper), side + " gripper open"),
            ((bdn, ['btnLeft']), (set_g, ['calibrate'], gripper), "right calibrate")
            ])
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
        if len(limb_cmd):
            limb.set_joint_positions(limb_cmd)
            limb_cmd.clear()
        rate.sleep()
    return False


def main():
    """SDK Joint Position Example: Joystick Control

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
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position joystick example"
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
    rospy.init_node("sdk_joint_position_joystick")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()

    map_joystick(joystick, args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
