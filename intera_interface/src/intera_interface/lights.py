# Copyright (c) 2016, Rethink Robotics
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
from intera_io import IODeviceInterface


class Lights(object):
    """
    Interface class for the lights on the Intera robots.
    """

    def __init__(self):
        """
        Constructor.

        """
        self._lights_io = IODeviceInterface("robot", "robot")

    def list_all_lights(self):
        """
        Returns a list of strings describing all available lights

        @rtype: list
        @return: a list of string representing light names
                 Each light name of the following format:
                 '<assembly>_<color>_light'
        """
        return [name for name in self._lights_io.list_signal_names() if "light" in name]

    def set_light_state(self, name, on=True):
        """
        Sets the named light the desired state (on=True, off=False)

        @type name: str
        @param name: Light name of the following format:
                     '<assembly>_<color>_light'
        @type on: bool
        @param on: value to set the light (on=True, off=False)
        """
        self._lights_io.set_signal_value(name, on)

    def get_light_state(self, name):
        """
        Returns a boolean describing whether the requested light is 'ON'.
        (True: on, False: off)

        @type name: str
        @param name: Light name of the following format:
                     '<assembly>_<color>_light'
        @rtype: bool
        @return:  a variable representing light state: (True: on, False: off)
        """
        return self._lights_io.get_signal_value(name)

