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

        @rtype: list [str]
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

        @rtype: bool
        @return: True if the light state is set, False otherwise
        """
        return self._lights_io.set_signal_value(name, on)

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
