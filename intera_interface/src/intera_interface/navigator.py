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

import threading
import uuid

import rospy

import intera_dataflow
from intera_io import IODeviceInterface

class Navigator(object):
    """
    Interface class for a Navigator on the Intera Research Robot.

    Signals:
        button_square_changed   - OFF/CLICK/LONG_PRESS/DOUBLE_CLICK
        button_ok_changed
        button_back_changed
        button_show_changed
        button_triangle_changed
        button_circle_changed
        wheel_changed           - Wheel value

    """

    def __init__(self):
        """
        Constructor.

        """
        self._navigator_io = IODeviceInterface("robot", "navigator")
        self._button_lookup = {0:'OFF',        1:'CLICK',
                               2:'LONG_PRESS', 3:'DOUBLE_CLICK'}

    def list_all_items(self):
        """
        Returns a list of strings describing all available navigator items

        @rtype: list
        @return: a list of string representing navigator items
                 Each item name of the following format:
                 '<assembly>_button_<function>'
        """
        return self._navigator_io.list_signal_names()

    def get_wheel_state(self, wheel_name):
        """
        Current state of the wheel providing wheel name
        @type wheel_name: str
        @param wheel_name: the wheel name

        @rtype: uint
        @return: an integer representing how far the wheel has turned
        """
        return self._get_item_state(wheel_name)

    def get_button_state(self, button_name):
        """
        Current button state by providing button name
        @type button_name: str
        @param button_name: the button name

        @rtype: uint
        @return: an integer representing button values
                 Valid states:
                 {0:'OFF', 1:'CLICK', 2:'LONG_PRESS', 3:'DOUBLE_CLICK'}
        """
        return self._get_item_state(button_name)

    def button_string_lookup(self, button_value):
        """
        Returns strings corresponding to the button state.

        @type button_value: int
        @param button_value: the value to lookup

        @rtype: str
        @return: 'INVALID_VALUE' if out of range, or if valid:
                 {0:'OFF', 1:'CLICK', 2:'LONG_PRESS', 3:'DOUBLE_CLICK'}
        """
        if button_value in self._button_lookup:
            return self._button_lookup[button_value]
        else:
            return 'INVALID_VALUE'

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
        return self._navigator_io.register_callback(
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
        return self._navigator_io.deregister_callback(callback_id)

    def _get_item_state(self, item_name):
        return self._navigator_io.get_signal_value(item_name)

