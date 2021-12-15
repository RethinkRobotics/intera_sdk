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
SDK Joint Position Example: file playback
"""
import argparse
import sys
import rospy
import intera_interface
from intera_interface import CHECK_VERSION


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None

def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = list(zip(names[1:], line[1:]))
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    right_command = dict((key, command[key]) for key in list(command.keys())
                         if key[:-2] == 'right_')
    return (command, right_command, line)

def map_file(filename, limb, loops=1):
    """
    Loops through csv file

    @param filename: the file to play
    @param loops: number of times to loop
                  values < 0 mean 'infinite'

    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    """
    limb_interface = intera_interface.Limb(limb)
    has_gripper = False
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("Could not detect a gripper attached to the robot")

    rate = rospy.Rate(100)
    if has_gripper:
        if gripper.has_error():
            gripper.reboot()
        if not gripper.is_calibrated():
            gripper.calibrate()

    print("Playing back: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')

    l = 0
    # If specified, repeat the file playback 'loops' number of times
    while loops < 1 or l < loops:
        i = 0
        l += 1
        print("Moving to start position...")
        _cmd, cmd_start, _raw = clean_line(lines[1], keys)
        limb_interface.move_to_joint_positions(cmd_start)
        start_time = rospy.get_time()
        for values in lines[1:]:
            i += 1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                             (i, len(lines) - 1, l, loopstr))
            sys.stdout.flush()
            cmd, limb_cmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n Aborting - ROS shutdown")
                    return False
                if len(limb_cmd):
                    limb_interface.set_joint_positions(limb_cmd)
                if has_gripper and gripper.name in cmd:
                        gripper.set_position(cmd[gripper.name])
                rate.sleep()
        print()
    return True

def main():
    """RSDK Joint Position Example: File Playback

    Uses Joint Position Control mode to play back a series of
    recorded joint and gripper positions.

    Run the joint_recorder.py example first to create a recording
    file for use with this example. This example uses position
    control to replay the recorded positions in sequence.

    Note: This version of the playback example simply drives the
    joints towards the next position at each time stamp. Because
    it uses Position Control it will not attempt to adjust the
    movement speed to hit set points "on time".
    """
    epilog = """
Related examples:
  joint_recorder.py; joint_trajectory_file_playback.py.
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
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        "-a", "--arm", dest="arm", default=valid_limbs[0],
        choices=valid_limbs,
        help="Arm on which to run the joint position file playback example.")
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of times to loop the input file. 0=infinite.'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_file_playback")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")

    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_file(args.file, args.arm, args.loops)


if __name__ == '__main__':
    main()
