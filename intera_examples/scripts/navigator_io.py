#!/usr/bin/python2

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

import argparse
import sys

import rospy

import intera_interface


def echo_input(nav_name='right'):
    nav = intera_interface.Navigator()
    def back_pressed(v):
        print ("Button 'Back': {0}".format(nav.button_string_lookup(v)))

    def rethink_pressed(v):
        print ("Button 'Rethink': {0}".format(nav.button_string_lookup(v)))

    def circle_pressed(v):
        print ("Button 'Circle': {0}".format(nav.button_string_lookup(v)))

    def square_pressed(v):
        print ("Button 'Square': {0}".format(nav.button_string_lookup(v)))

    def x_pressed(v):
        print ("Button 'X': {0}".format(nav.button_string_lookup(v)))

    def ok_pressed(v):
        print ("Button 'OK': {0}".format(nav.button_string_lookup(v)))

    def wheel_moved(v):
        print ("Wheel value: {0}".format(v))

    nav.register_callback(back_pressed, '_'.join([nav_name, 'button_back']))
    nav.register_callback(rethink_pressed, '_'.join([nav_name, 'button_show']))
    nav.register_callback(circle_pressed, '_'.join([nav_name, 'button_circle']))
    nav.register_callback(square_pressed, '_'.join([nav_name, 'button_square']))
    nav.register_callback(x_pressed, '_'.join([nav_name, 'button_triangle']))
    nav.register_callback(ok_pressed, '_'.join([nav_name, 'button_ok']))
    nav.register_callback(wheel_moved, '_'.join([nav_name, 'wheel']))

    print ("Press input buttons on the right navigator, "
           "input will be echoed here.")

    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown() and i < 10:
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
