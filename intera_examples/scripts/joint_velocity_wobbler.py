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
import math
import random

import rospy

from std_msgs.msg import (
    UInt16,
)

import intera_interface

from intera_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = intera_interface.limb.Limb("right")
        self._right_joint_names = self._right_arm.joint_names()

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = random.uniform(0.1, 0.2)

            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func

        v_funcs = [make_v_func() for _ in self._right_joint_names]

        def make_cmd(joint_names, elapsed):
            return dict([(joint, v_funcs[i](elapsed))
                         for i, joint in enumerate(joint_names)])

        print("Wobbling. Press Ctrl-C to stop...")
        while not rospy.is_shutdown():
            self._pub_rate.publish(self._rate)
            elapsed = rospy.Time.now() - start
            cmd = make_cmd(self._right_joint_names, elapsed)
            self._right_arm.set_joint_velocities(cmd)
            rate.sleep()


def main():
    """Intera RSDK Joint Velocity Example: Wobbler

    Commands joint velocities of randomly parameterized cosine waves
    to each joint. Demonstrates Joint Velocity Control Mode.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_velocity_wobbler")

    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    wobbler.wobble()

    print("Done.")

if __name__ == '__main__':
    main()
