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
