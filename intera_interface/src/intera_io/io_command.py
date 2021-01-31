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

import json
import rospy

from intera_core_msgs.msg import IOComponentCommand


class IOCommand(object):
    '''
    Container for a generic IO command
    '''
    def __init__(self, op, args=None, time=None, now=False):
        self.time = rospy.Time() if time is None else time
        if now:
            self.time = rospy.Time.now()
        self.op = op
        self.args = args if args else {}

    def __str__(self):
        return str({"time": self.time, "op": self.op, "args": self.args})

    def as_msg(self, now=None):
        """
        Returns a properly formatted ROS Message that can be used in a Publisher.

        Primarily "stringify"s the json '.args' field and creates an actual
        IOComponentCommand ROS Message. Also sets the timestamp if not currently set.

        @type now: bool|None
        @param now: sets the '.time' field of the ROS Message. True: set time to now();
            False: reset time to empty (0, 0); Default: set time to now if unset or empty

        @rtype: IOComponentCommand
        @return: proper ROS Message to Publish
        """
        if now == None:       # use time we have OR set to now if we don't have time!
            self.time = self.time if not self.time.is_zero() else rospy.Time.now()
        elif now == True:     # set time to now
            self.time = rospy.Time.now()
        elif now == False:    # reset time to empty (0,0)
            self.time = rospy.Time()

        return IOComponentCommand(
            time = self.time,
            op = self.op,
            args = json.dumps(self.args)
        )

class SetCommand(IOCommand):
    '''
    Container for a port or signal set command
    '''
    def __init__(self, args=None):
        super(SetCommand, self).__init__('set', args)

    def _set(self, components, component_name,
             data_type, dimensions, *component_value):
        '''
        add a set component command
        '''
        self.args.setdefault(components, {})
        self.args[components][component_name] = {
            'format' : {'type' : data_type},
            'data'   : [val for val in component_value]
        }
        if dimensions > 1:
            self.args[components][component_name]['format']['dimensions'] = [dimensions]

    def set_signal(self, signal_name, data_type, *signal_value):
        '''
        add a set signal command
        '''
        dimensions = len(signal_value)
        self._set('signals', signal_name, data_type, dimensions, *signal_value)
        return self

    def set_port(self, port_name, data_type, *port_value):
        '''
        add a set port command
        '''
        dimensions = len(port_value)
        self._set('ports', port_name, data_type, dimensions, *port_value)
        return self
