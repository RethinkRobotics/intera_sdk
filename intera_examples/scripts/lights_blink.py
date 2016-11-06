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

import argparse

import rospy

from intera_interface import Lights


def test_light_interface(light_name='head_green_light'):
    """Blinks a desired Light on then off."""
    l = Lights()
    rospy.loginfo("All available lights on this robot:\n{0}\n".format(
                                               ', '.join(l.list_all_lights())))
    rospy.loginfo("Blinking Light: {0}".format(light_name))
    on_off = lambda x: 'ON' if l.get_light_state(x) else 'OFF'
    rospy.loginfo("Initial state: {0}".format(on_off(light_name)))
    # turn on light
    l.set_light_state(light_name, True)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # turn off light
    l.set_light_state(light_name, False)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # turn on light
    l.set_light_state(light_name, True)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # reset output
    l.set_light_state(light_name, False)
    rospy.sleep(1)
    rospy.loginfo("Final state: {0}".format(on_off(light_name)))


def main():
    """Intera SDK Lights Example: Blink

    Toggles the Lights interface on then off again
    while printing the state at each step. Simple demonstration
    of using the intera_interface.Lights class.

    Run this example with default arguments and watch the green
    light on the head blink on and off while the console
    echos the state. Use the light names from Lights.list_all_lights()
    to change lights to toggle.
    """
    epilog = """ Intera Interface Lights """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        '-l', '--light_name', dest='light_name',
        default='head_green_light',
        help=('name of Light component to use'
              ' (default: head_green_light)')
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('sdk_lights_blink', anonymous=True)
    test_light_interface(args.light_name)

if __name__ == '__main__':
    main()
