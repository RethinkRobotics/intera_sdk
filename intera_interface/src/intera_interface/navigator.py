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
from io_interface import IODeviceInterface

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
        self._threads = dict()
        self._signal_items = dict()
        self._callback_functions = dict()


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

        @rtype: str
        @return: a string representing the current button state
                 Valid states:
                 ['OFF', 'CLICK', 'LONG_PRESS', 'DOUBLE_CLICK']
        """
        return self._get_item_state(button_name)

    def register_callback(self, callback_function, item_name, poll_rate=10):
        """
        Registers a supplied callback to a change in state of supplied
        item_name's value. Spawns a thread that will call the callback with
        the updated value.

        @type: function
        @param: function handle for callback function
        @type: str
        @param: the item name (button or wheel) to poll for value change
        @type: int
        @param: the rate at which to poll for a value change (in a separate
                thread)

        @rtype: str
        @return: callback_id retuned if the callback was registered, and an
                 empty string if the requested item_name does not exist in the
                 Navigator
        """
        if item_name in self.list_all_items():
            callback_id = uuid.uuid4()
            self._signal_items[callback_id] = intera_dataflow.Signal()
            def signal_spinner():
                old_state = self._get_item_state(item_name)
                r = rospy.Rate(poll_rate)
                while not rospy.is_shutdown():
                  new_state = self._get_item_state(item_name)
                  if new_state != old_state:
                      self._signal_items[callback_id](new_state)
                  old_state = new_state
                  r.sleep()
            self._signal_items[callback_id].connect(callback_function)
            t = threading.Thread(target=signal_spinner)
            t.daemon = True
            t.start()
            self._threads[callback_id] = t
            self._callback_functions[callback_id] = callback_function
            return callback_id
        else:
            return str()

    def deregister_callback(self, callback_id):
        """
        Deregisters a callback based on the supplied callback_id.

        @type: str
        @param: the callback_id string to deregister

        @rtype: bool
        @return: returns bool True if the callback was successfully
                 deregistered, and False otherwise.
        """
        if callback_id in self._threads.keys():
            self._signal_items[callback_id].disconnect(
                              self._callback_functions[callback_id])
            return True
        else:
            return False

    def _get_item_state(self, item_name):
        state = self._navigator_io.get_signal_value(item_name)
        state = self._button_lookup[state] if 'wheel' not in item_name else state
        return state
