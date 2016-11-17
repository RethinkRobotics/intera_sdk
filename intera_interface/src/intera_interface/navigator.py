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
        @type: str
        @param: the wheel name

        @rtype: uint
        @return: an integer representing how far the wheel has turned
        """
        return self._get_item_state(wheel_name)

    def get_button_state(self, button_name):
        """
        Current button state by providing button name
        @type: str
        @param: the button name

        @rtype: uint
        @return: an integer representing button values
                 Valid states:
                 {0:'OFF', 1:'CLICK', 2:'LONG_PRESS', 3:'DOUBLE_CLICK'}
        """
        return self._get_item_state(button_name)

    def button_string_lookup(self, button_value):
        """
        Returns strings corresponding to the button state.

        @type: int
        @param: the value to lookup

        @type: str
        @param: 'INVALID_VALUE' if out of range, or if valid:
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
        return self._navigator_io.register_callback(
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
        return self._navigator_io.deregister_callback(callback_id)

    def _get_item_state(self, item_name):
        return self._navigator_io.get_signal_value(item_name)

