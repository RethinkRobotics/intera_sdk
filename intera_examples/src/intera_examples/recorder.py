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
    def __init__(self, filename, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self.gripper_name = '_'.join([side, 'gripper'])
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_right = intera_interface.Limb(side)
        try:
            self._gripper = intera_interface.Gripper(side)
        except:
            self.has_gripper = False
            rospy.logerr("Could not initalize the gripper.")
        else:
            self.has_gripper = True

        # Verify Gripper Have No Errors and are Calibrated
        if self.has_gripper:
            self._gripper.set_holding_force(0.0)
            if self._gripper.has_error():
                self._gripper.reboot()                
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

        self._cuff = intera_interface.Cuff(side)

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
                temp_str = '' if self.has_gripper else '\n'
                f.write(','.join([j for j in joints_right]) + ',' + temp_str)
                if self.has_gripper:
                    f.write(self.gripper_name+'\n')
                while not self.done():
                    if self.has_gripper:
                        if self._cuff.upper_button():
                            self._gripper.open()
                        elif self._cuff.lower_button():
                            self._gripper.close()
                    angles_right = [self._limb_right.joint_angle(j)
                                    for j in joints_right]
                    f.write("%f," % (self._time_stamp(),))
                    f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
                    if self.has_gripper:
                        f.write(str(self._gripper.get_position()) + '\n')
                    self._rate.sleep()
