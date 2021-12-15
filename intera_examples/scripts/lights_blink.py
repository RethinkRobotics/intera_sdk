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

import rospy

from intera_interface import Lights


def test_light_interface(light_name='head_green_light'):
    """Blinks a desired Light on then off."""
    l = Lights()
    rospy.loginfo("All available lights on this robot:\n{0}\n".format(
                                               ', '.join(l.list_all_lights())))
    rospy.loginfo("Blinking Light: {0}".format(light_name))
    initial_state = l.get_light_state(light_name)
    on_off = lambda x: 'ON' if l.get_light_state(x) else 'OFF'
    rospy.loginfo("Initial state: {0}".format(on_off(light_name)))
    # invert light
    state = not initial_state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # invert light
    state = not state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # invert light
    state = not state
    l.set_light_state(light_name, state)
    rospy.sleep(1)
    rospy.loginfo("New state: {0}".format(on_off(light_name)))
    # reset output
    l.set_light_state(light_name, initial_state)
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
