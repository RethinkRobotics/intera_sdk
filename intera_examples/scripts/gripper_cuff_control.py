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

from intera_interface import (
    Gripper,
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
            self._gripper = Gripper(arm)
            if not (self._gripper.is_calibrated() or
                    self._gripper.calibrate() == True):
                rospy.logerr("({0}_gripper) calibration failed.".format(
                               self._gripper.name))
                raise
            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))
            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except:
            self._gripper = None
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)


    def _open_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            self._gripper.open()
            if self._lights:
                self._set_lights('red', False)
                self._set_lights('green', True)

    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
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
