# Copyright (c) 2013-2017, Rethink Robotics Inc.
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

class IOCommand(object):
    '''
    Container for a generic io command
    '''
    def __init__(self, op, args=None):
        self.op = op
        self.args = args if args else {}

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
