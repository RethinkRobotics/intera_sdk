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
import intera_dataflow
from io_interface import IODeviceInterface
from intera_core_msgs.msg import IODeviceConfiguration


class Cuff(object):
    """
    Interface class for cuff on the Intera Research Robot.
    """

    def __init__(self):
        """
        """
        self.device = None
        self.name = "cuff"
        self.cuff_config_sub = rospy.Subscriber('/io/robot/cuff/config', IODeviceConfiguration, self._config_callback)
        # Wait for the cuff status to be true
        intera_dataflow.wait_for(
            lambda: not self.device is None, timeout=5.0,
            timeout_msg=("Failed to get cuff state.")
            )

        self.cuff_io = IODeviceInterface("robot", self.name)

    def _config_callback(self, msg):
        """
        config topic callback
        """
        if msg.device != []:
            if str(msg.device.name) == self.name:
                self.device = msg

    def lower_button(self):
        """
        Returns a list within a integer value describing whether the lower button on cuff is pressed.
        ([0]: no pressing, [1]: pressing )
        @rtype: list
        """
        return self.cuff_io.get_signal_value("right_button_lower")

    def upper_button(self):
        """
        Returns a list within a integer value describing whether the upper button on cuff is pressed.
        ([0]: no pressing, [1]: pressing )
        @rtype: list
        """
        return self.cuff_io.get_signal_value("right_button_upper")

    def cuff(self):
        """
        Returns a list within a integer value describing whether the cuff is pressed.
        ([0]: no pressing, [1]: pressing )
        @rtype: list
        """
        return self.cuff_io.get_signal_value("has_error")
