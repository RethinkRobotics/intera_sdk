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

        @type callback_function: function
        @param callback_function: function handle for callback function
        @type signal_name: str
        @param signal_name: the name of the signal to poll for value change
        @type poll_rate: int
        @param poll_rate: the rate at which to poll for a value change (in a separate
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

        @type callback_id: str
        @param callback_id: the callback_id string to deregister

        @rtype: bool
        @return: returns bool True if the callback was successfully
                 deregistered, and False otherwise.
        """
        return self._cuff_io.deregister_callback(callback_id)
