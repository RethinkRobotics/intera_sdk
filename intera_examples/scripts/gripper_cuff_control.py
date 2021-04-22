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

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Lights,
    Cuff,
    RobotParams,
)


class GripperConnect(object):
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the Navigator callback feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._cuff = Cuff(limb=arm)
        # connect callback fns to signals
        self._lights = None
        if lights:
            self._lights = Lights()
            self._cuff.register_callback(self._light_action,
                                         '{0}_cuff'.format(arm))
        try:
            self._gripper = get_current_gripper_interface()
            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

            if self._is_clicksmart:
                if self._gripper.needs_init():
                    self._gripper.initialize()
            else:
                if not (self._gripper.is_calibrated() or
                        self._gripper.calibrate() == True):
                    raise
            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))

            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except:
            self._gripper = None
            self._is_clicksmart = False
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)

    def _open_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
            else:
                self._gripper.open()
            if self._lights:
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', True)
            else:
                self._gripper.close()
            if self._lights:
                self._set_lights('green', False)
                self._set_lights('red', True)

    def _light_action(self, value):
        if value:
            rospy.logdebug("cuff grasp triggered")
        else:
            rospy.logdebug("cuff release triggered")
        if self._lights:
            self._set_lights('red', False)
            self._set_lights('green', False)
            self._set_lights('blue', value)

    def _set_lights(self, color, value):
        self._lights.set_light_state('head_{0}_light'.format(color), on=bool(value))
        self._lights.set_light_state('{0}_hand_{1}_light'.format(self._arm, color),
                                                                 on=bool(value))

def main():
    """SDK Gripper Button Control Example

    Connects cuff buttons to gripper open/close commands:
        'Circle' Button    - open gripper
        'Dash' Button      - close gripper
        Cuff 'Squeeze'     - turn on Nav lights

    Run this example in the background or in another terminal
    to be able to easily control the grippers by hand while
    using the robot. Can be run in parallel with other code.
    """
    rp = RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    if len(valid_limbs) > 1:
        valid_limbs.append("all_limbs")
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('-g', '--gripper', dest='gripper', default=valid_limbs[0],
                        choices=[valid_limbs],
                        help='gripper limb to control (default: both)')
    parser.add_argument('-n', '--no-lights', dest='lights',
                        action='store_false',
                        help='do not trigger lights on cuff grasp')
    parser.add_argument('-v', '--verbose', dest='verbosity',
                        action='store_const', const=rospy.DEBUG,
                        default=rospy.INFO,
                        help='print debug statements')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('sdk_gripper_cuff_control_{0}'.format(args.gripper),
                    log_level=args.verbosity)

    arms = (args.gripper,) if args.gripper != 'all_limbs' else valid_limbs[:-1]
    grip_ctrls = [GripperConnect(arm, args.lights) for arm in arms]

    print("Press cuff buttons for gripper control. Spinning...")
    rospy.spin()
    print("Gripper Button Control Finished.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
