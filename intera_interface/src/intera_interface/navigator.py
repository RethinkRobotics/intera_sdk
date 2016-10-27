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

from intera_core_msgs.msg import NavigatorState
from intera_interface import robot_params

class Navigator(object):
    """
    Interface class for a Navigator on the Intera Research Robot.

    Inputs:
        Button ok       - press wheel
        Button back     - press back
        Button show     - press rethink
        Button triangle - press triangle
        Button circle   - press circle
        Button square   - press square
        Scroll wheel - 0-255

    Signals:
        button_ok_changed       - True/False
        button_back_changed     - True/False
        button_show_changed     - True/False
        button_triangle_changed - True/False
        button_circle_changed   - True/False
        button_square_changed   - True/False
        wheel_changed   - New wheel value

    Valid identifiers:
        right, head, torso
    """

    def __init__(self, location):
        """
        Constructor.

        @type location: str
        @param location: body location (prefix) of Navigator to control.

        Valid locations are: right, head, torso
        """
        self.locations = robot_params.RobotParams().get_robot_assemblies()
        if not location in self.locations:
            raise AttributeError("Invalid Navigator name '%s'" % (location,))
        self._id = location
        self._state = None
        self.button_ok_changed = intera_dataflow.Signal()
        self.button_back_changed = intera_dataflow.Signal()
        self.button_show_changed = intera_dataflow.Signal()
        self.button_triangle_changed = intera_dataflow.Signal()
        self.button_circle_changed = intera_dataflow.Signal()
        self.button_square_changed = intera_dataflow.Signal()
        self.wheel_changed = intera_dataflow.Signal()

        nav_state_topic = 'robot/navigators/{0}_navigator/state'.format(self._id)
        self._state_sub = rospy.Subscriber(
            nav_state_topic,
            NavigatorState,
            self._on_state)

        init_err_msg = ("Navigator init failed to get current state from %s" %
                        (nav_state_topic,))
        intera_dataflow.wait_for(lambda: self._state != None,
                                 timeout_msg=init_err_msg)

    @property
    def wheel(self):
        """
        Current state of the wheel
        """
        return self._state.wheel

    @property
    def button_state(self, button_name):
        """
        Current button state by providing button name
        @type: str
        @param: the button name
        """
        return self._state.buttons[self._state.button_names.index(button_name)]

    def _on_state(self, msg):
        if not self._state:
            self._state = msg
            return

        old_state = self._state
        self._state = msg

        buttons = [self.button_ok_changed,
                   self.button_back_changed,
                   self.button_show_changed,
                   self.button_triangle_changed,
                   self.button_circle_changed,
                   self.button_square_changed
                   ]
        for i, signal in enumerate(buttons):
            if old_state.buttons[i] != msg.buttons[i]:
                signal(msg.buttons[i])

        if old_state.wheel != msg.wheel:
            diff = msg.wheel - old_state.wheel
            if abs(diff % 256) < 127:
                self.wheel_changed(diff % 256)
            else:
                self.wheel_changed(diff % (-256))

