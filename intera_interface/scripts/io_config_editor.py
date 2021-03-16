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
SDK Read / Write ClickSmart (EndEffector) Configuration File
"""

import argparse
import json

import rospy

import intera_interface


def save_config(device, filename):
    """
    Saves the current End Effector configuration on a ClickSmart device to the
    given local file as JSON.

    @type device: SimpleClickSmartGripper
    @param device: instance of ClickSmart for current gripper
    @type filename: str
    @param filename: path to output file where config will be written
                     (will overwrite file if already exists)
    """
    # get config from ClickSmart device
    try:
        config_msg = device.config
    except:
        # try epg structure
        config_msg = device.gripper_io.config
    device_name = config_msg.device.name
    config = config_msg.device.config

    # save config to file
    if filename:
        # parse string so output can be nicely formatted
        json_config = json.loads(config)
        with open(filename, 'w') as f:
            json.dump(json_config, f, ensure_ascii=False, sort_keys=True,
                indent=2, separators=(",", ": "))
    else:
        rospy.logerr("Bad filename: '{}'".format(filename))

def load_config(device, filename):
    """
    Read a JSON End Effector configuration from a local file and load it onto
    a ClickSmart attached to the robot. Reconfigures the ClickSmart and
    stores the configuration on the ClickSmart plate.

    @type device: SimpleClickSmartGripper
    @param device: instance of ClickSmart for current gripper
    @type filename: str
    @param filename: path to input file containing the JSON EE config
    """
    with open(filename, 'r') as f:
        config_data = json.load(f)

    device.send_configuration(config_data)


def main():
    """Save / Load EndEffector Config utility

    Read current config off of ClickSmart and save to file.
    Or load config from file and reconfigure and save it to ClickSmart device.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)

    parser.add_argument(
        '-s', '--save', metavar='PATH',
        help='save current EE config to given file'
    )
    parser.add_argument(
        '-l', '--load', metavar='PATH',
        help='load config from given file onto EE'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('ee_config_editor', anonymous=True)

    ee = intera_interface.get_current_gripper_interface()
    if not ee:
        rospy.logerr("Could not detect an attached EndEffector!")
        return

    if args.save:
        rospy.loginfo("Saving EE config to {}".format(args.save))
        save_config(ee, args.save)

    if args.load:
        rospy.loginfo("Loading config and writing config to ClickSmart from {}".format(args.load))
        load_config(ee, args.load)

    def clean_shutdown():
        print("\nExiting example...")

    rospy.on_shutdown(clean_shutdown)


if __name__ == '__main__':
    main()
