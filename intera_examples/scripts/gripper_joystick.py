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
SDK Gripper Example: joystick
"""
import argparse

import rospy

import intera_interface
import intera_external_devices


def map_joystick(joystick, limb):
    """
    maps joystick input to gripper commands

    @param joystick: an instance of a Joystick
    """
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
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

    def offset_position(offset_pos):
        cmd_pos = max(min(gripper.get_position() + offset_pos, gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))

    def update_velocity(offset_vel):
        cmd_speed = max(min(gripper.get_cmd_velocity() + offset_vel, gripper.MAX_VELOCITY), gripper.MIN_VELOCITY)
        gripper.set_cmd_velocity(cmd_speed)
        print("commanded velocity set to {0} m/s".format(cmd_speed))


    # decrease position dead_zone
    original_deadzone = gripper.get_dead_zone()
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    num_steps = 8.0
    percent_delta = 1.0 / num_steps
    position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta
    velocity_increment = (gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) * percent_delta


    bindings_list = []
    bindings = (
        #(test, command, description)
        ((bdn, ['btnLeft']), (gripper.reboot, []), "reboot"),
        ((bdn, ['btnUp']), (gripper.calibrate, []), "calibrate"),
        ((bdn, ['leftTrigger']), (gripper.close, []), "close"),
        ((bup, ['leftTrigger']), (gripper.open, []), "open (release)"),
        ((bdn, ['leftBumper']), (gripper.stop, []), "stop"),
        ((jlo, ['leftStickVert']), (offset_position, [-position_increment]),
                                    "decrease position"),
        ((jhi, ['leftStickVert']), (offset_position, [position_increment]),
                                     "increase position"),
        ((jlo, ['rightStickVert']), (update_velocity, [-velocity_increment]),
                                    "decrease commanded velocity"),
        ((jhi, ['rightStickVert']), (update_velocity, [velocity_increment]),
                                    "increase commanded velocity"),

        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press <Start> button for help; Ctrl-C to stop...")
    while not rospy.is_shutdown():
        # test each joystick condition and call binding cmd if true
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                print(doc)
                cmd[0](*cmd[1])
        rate.sleep()
    rospy.signal_shutdown("Example finished.")


def main():
    """SDK Gripper Example: Joystick Control

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
        '-j', '--joystick', required=True, choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the gripper joystick example"
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
    rospy.init_node("sdk_gripper_joystick")

    map_joystick(joystick, args.limb)


if __name__ == '__main__':
    main()
