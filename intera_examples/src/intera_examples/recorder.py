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

import rospy

import intera_interface

from intera_interface import CHECK_VERSION


class JointRecorder(object):
    def __init__(self, filename, rate):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_right = intera_interface.Limb("right")
        #####################################################
        # TODO: Fix gripper.py then enable gripper control. #
        #####################################################
        #self._gripper_right = intera_interface.Gripper("right", CHECK_VERSION)
        #self._io_right_lower = intera_interface.DigitalIO('right_lower_button')
        #self._io_right_upper = intera_interface.DigitalIO('right_upper_button')
        #####################################################
        # TODO: Fix gripper.py then enable gripper control. #
        #####################################################
        # Verify Gripper Have No Errors and are Calibrated
        #if self._gripper_right.error():
        #    self._gripper_right.reset()
        #if (not self._gripper_right.calibrated() and
        #    self._gripper_right.type() != 'custom'):
        #    self._gripper_right.calibrate()

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.

        If a file exists, the function will overwrite existing file.
        """
        if self._filename:
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                # TODO: Once have gripper data, rm '\n' in line below.
                f.write(','.join([j for j in joints_right]) + ',' + '\n')
                #f.write('right_gripper\n')

                while not self.done():
                    #####################################################
                    # TODO: Fix gripper.py then enable gripper control. #
                    #####################################################
                    # Look for gripper button presses
                    #if self._io_right_lower.state:
                    #    self._gripper_right.open()
                    #elif self._io_right_upper.state:
                    #    self._gripper_right.close()

                    angles_right = [self._limb_right.joint_angle(j)
                                    for j in joints_right]

                    f.write("%f," % (self._time_stamp(),))
                    # TODO: Once have gripper data, rm '\n' in line below.
                    f.write(','.join([str(x) for x in angles_right]) + ','+ '\n')
                    #####################################################
                    # TODO: Fix gripper.py then enable gripper control. #
                    #####################################################
                    #f.write(str(self._gripper_right.position()) + '\n')

                    self._rate.sleep()