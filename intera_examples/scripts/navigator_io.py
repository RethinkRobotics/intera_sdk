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

import argparse
import sys

import rospy

import intera_interface


def echo_input(nav_name='right'):
    nav = intera_interface.Navigator()
    def back_pressed(v):
        rospy.loginfo("Button 'Back': {0}".format(nav.button_string_lookup(v)))

    def rethink_pressed(v):
        rospy.loginfo ("Button 'Rethink': {0}".format(nav.button_string_lookup(v)))

    def circle_pressed(v):
        rospy.loginfo ("Button 'Circle': {0}".format(nav.button_string_lookup(v)))

    def square_pressed(v):
        rospy.loginfo ("Button 'Square': {0}".format(nav.button_string_lookup(v)))

    def x_pressed(v):
        rospy.loginfo ("Button 'X': {0}".format(nav.button_string_lookup(v)))

    def ok_pressed(v):
        rospy.loginfo ("Button 'OK': {0}".format(nav.button_string_lookup(v)))

    def wheel_moved(v):
        rospy.loginfo ("Wheel value: {0}".format(v))

    nav.register_callback(back_pressed, '_'.join([nav_name, 'button_back']))
    nav.register_callback(rethink_pressed, '_'.join([nav_name, 'button_show']))
    nav.register_callback(circle_pressed, '_'.join([nav_name, 'button_circle']))
    nav.register_callback(square_pressed, '_'.join([nav_name, 'button_square']))
    nav.register_callback(x_pressed, '_'.join([nav_name, 'button_triangle']))
    nav.register_callback(ok_pressed, '_'.join([nav_name, 'button_ok']))
    nav.register_callback(wheel_moved, '_'.join([nav_name, 'wheel']))

    print ("Press input buttons on the right navigator, "
           "input will be echoed here.")

    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown() and i < 100:
        rate.sleep()
        i += 1


def main():
    """SDK Navigator Example

    Demonstrates Navigator by echoing input values from wheels and
    buttons.

    Uses the intera_interface.Navigator class to demonstrate an
    example of using the register_callback feature.
        
     Shows Navigator input of the arm for 10 seconds.
    """
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-n", "--navigator", dest="nav_name", default="right",
        choices=["right", "head"],
        help='Navigator on which to run example'
        )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('sdk_navigator', anonymous=True)
    echo_input(args.nav_name)
    return 0

if __name__ == '__main__':
    sys.exit(main())
