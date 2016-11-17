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
from intera_io import IODeviceInterface
from intera_core_msgs.msg import IODeviceConfiguration
from robot_params import RobotParams


class Cuff(object):
    """
    Interface class for cuff on the Intera robots.
    """

    def __init__(self, limb="right"):
        """
        Constructor.

        @type limb: str
        @param limb: limb side to interface
        """
        params = RobotParams()
        limb_names = params.get_limb_names()
        if limb not in limb_names:
            rospy.logerr("Cannot detect Cuff's limb {0} on this robot."
                         " Valid limbs are {1}. Exiting Cuff.init().".format(
                                                            limb, limb_names))
            return
        self.limb = limb
        self.device = None
        self.name = "cuff"
        self.cuff_config_sub = rospy.Subscriber('/io/robot/cuff/config', IODeviceConfiguration, self._config_callback)
        # Wait for the cuff status to be true
        intera_dataflow.wait_for(
            lambda: not self.device is None, timeout=5.0,
            timeout_msg=("Failed find cuff on limb '{0}'.".format(limb))
            )
        self._cuff_io = IODeviceInterface("robot", self.name)

    def _config_callback(self, msg):
        """
        config topic callback
        """
        if msg.device != []:
            if str(msg.device.name) == self.name:
                self.device = msg.device

    def lower_button(self):
        """
        Returns a boolean describing whether the lower button on cuff is pressed.

        @rtype: bool
        @return: a variable representing button state: (True: pressed, False: unpressed)
        """
        return bool(self._cuff_io.get_signal_value('_'.join([self.limb, "button_lower"])))

    def upper_button(self):
        """
        Returns a boolean describing whether the upper button on cuff is pressed.
        (True: pressed, False: unpressed)
        @rtype: bool
        @return:  a variable representing button state: (True: pressed, False: unpressed)
        """
        return bool(self._cuff_io.get_signal_value('_'.join([self.limb, "button_upper"])))

    def cuff_button(self):
        """
        Returns a boolean describing whether the cuff button on cuff is pressed.
        (True: pressed, False: unpressed)
        @rtype: bool
        @return:  a variable representing cuff button state: (True: pressed, False: unpressed)
        """
        return bool(self._cuff_io.get_signal_value('_'.join([self.limb, "cuff"])))

    def register_callback(self, callback_function, signal_name, poll_rate=10):
        """
        Registers a supplied callback to a change in state of supplied
        signal_name's value. Spawns a thread that will call the callback with
        the updated value.

        @type: function
        @param: function handle for callback function
        @type: str
        @param: the name of the signal to poll for value change
        @type: int
        @param: the rate at which to poll for a value change (in a separate
                thread)

        @rtype: str
        @return: callback_id retuned if the callback was registered, and an
                 empty string if the requested signal_name does not exist in the
                 Navigator
        """
        return self._cuff_io.register_callback(
                                 callback_function=callback_function,
                                 signal_name=signal_name,
                                 poll_rate=poll_rate)

    def deregister_callback(self, callback_id):
        """
        Deregisters a callback based on the supplied callback_id.

        @type: str
        @param: the callback_id string to deregister

        @rtype: bool
        @return: returns bool True if the callback was successfully
                 deregistered, and False otherwise.
        """
        return self._cuff_io.deregister_callback(callback_id)
