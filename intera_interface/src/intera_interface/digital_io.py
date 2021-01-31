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

import errno

import rospy

import intera_dataflow

from intera_core_msgs.msg import (
    DigitalIOState,
    DigitalOutputCommand,
)


class DigitalIO(object):
    """
    DEPRECATION WARNING: This interface will likely be removed in
    the future. Transition to using the IO Framework and the wrapper
    classes: gripper.py, cuff.py, camera.py

    Interface class for a simple Digital Input and/or Output on the
    Intera robots.

    Input
      - read input state
    Output
      - turn output On/Off
      - read current output state
    """
    def __init__(self, component_id):
        """
        Constructor.

        @param component_id: unique id of the digital component
        """
        self._id = component_id
        self._component_type = 'digital_io'
        self._is_output = False
        self._state = None

        self.state_changed = intera_dataflow.Signal()

        type_ns = '/robot/' + self._component_type
        topic_base = type_ns + '/' + self._id

        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            DigitalIOState,
            self._on_io_state)

        intera_dataflow.wait_for(
            lambda: self._state != None,
            timeout=2.0,
            timeout_msg="Failed to get current digital_io state from %s" \
            % (topic_base,),
        )

        # check if output-capable before creating publisher
        if self._is_output:
            self._pub_output = rospy.Publisher(
                type_ns + '/command',
                DigitalOutputCommand,
                queue_size=10)

    def _on_io_state(self, msg):
        """
        Updates the internally stored state of the Digital Input/Output.
        """
        new_state = (msg.state == DigitalIOState.PRESSED)
        if self._state is None:
            self._is_output = not msg.isInputOnly
        old_state = self._state
        self._state = new_state

        # trigger signal if changed
        if old_state is not None and old_state != new_state:
            self.state_changed(new_state)

    @property
    def is_output(self):
        """
        Accessor to check if IO is capable of output.
        """
        return self._is_output

    @property
    def state(self):
        """
        Current state of the Digital Input/Output.
        """
        return self._state

    @state.setter
    def state(self, value):
        """
        Control the state of the Digital Output. (is_output must be True)

        @type value: bool
        @param value: new state to output {True, False}
        """
        self.set_output(value)

    def set_output(self, value, timeout=2.0):
        """
        Control the state of the Digital Output.

        Use this function for finer control over the wait_for timeout.

        @type value: bool
        @param value: new state {True, False} of the Output.
        @type timeout: float
        @param timeout: Seconds to wait for the io to reflect command.
                        If 0, just command once and return. [0]
        """
        if not self._is_output:
            raise IOError(errno.EACCES, "Component is not an output [%s: %s]" %
                (self._component_type, self._id))
        cmd = DigitalOutputCommand()
        cmd.name = self._id
        cmd.value = value
        self._pub_output.publish(cmd)

        if not timeout == 0:
            intera_dataflow.wait_for(
                test=lambda: self.state == value,
                timeout=timeout,
                rate=100,
                timeout_msg=("Failed to command digital io to: %r" % (value,)),
                body=lambda: self._pub_output.publish(cmd)
            )
