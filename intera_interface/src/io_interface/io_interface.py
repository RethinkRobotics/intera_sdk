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
import sys
import json
from threading import Lock
from intera_dataflow import Signal

from intera_core_msgs.msg import IONodeConfiguration, IODeviceConfiguration, \
                                 IONodeStatus, IODeviceStatus, \
                                 IOComponentCommand

def _time_changed(time1, time2):
    """
    return true if the times are different
    """
    return (time1.secs != time2.secs) or (time1.nsecs != time2.nsecs)

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

class IOInterface(object):
    """
    Base class for IO interfaces.
    """
    def __init__(self, path_root, config_msg_type, status_msg_type):
        self._path = path_root
        self.config_mutex = Lock()
        self.state_mutex = Lock()
        self.cmd_times = []
        self.state = None
        self.config = None
        self.state_changed = Signal()
        self.config_changed = Signal()

        self._config_sub = rospy.Subscriber(self._path + "/config",
                                            config_msg_type,
                                            self.handle_config)
        self._state_sub = rospy.Subscriber(self._path + "/state",
                                           status_msg_type,
                                           self.handle_state)
        self._command_pub = rospy.Publisher(self._path + "/command",
                                            IOComponentCommand, queue_size=10)

        rospy.loginfo("Making new IOInterface on %s" % (self._path,))

    def invalidate_config(self):
        """
        mark the config topic data as invalid
        """
        with self.config_mutex:
            self.config.time.secs = 0

    def invalidate_state(self):
        """
        mark the state topic data as invalid
        """
        with self.state_mutex:
            self.state.time.secs = 0

    def is_config_valid(self):
        """
        return true if the config topic data is valid
        """
        return self.config.time.secs != 0

    def is_state_valid(self):
        """
        return true if the state topic data is valid
        """
        return self.state.time.secs != 0

    def is_valid(self):
        """
        return true if both the state and config topic data are valid
        """
        return self.is_config_valid() and self.is_state_valid()

    def revalidate(self, timeout, invalidate_state=True, invalidate_config=True):
        """
        invalidate the state and config topics, then wait up to timeout
        seconds for them to become valid again.
        return true if both the state and config topic data are valid
        """
        if invalidate_state:
            self.invalidate_state()
        if invalidate_config:
            self.invalidate_config()
        timeout_time = rospy.Time.now() + rospy.Duration(timeout)
        while not self.is_valid() and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if timeout_time < rospy.Time.now():
                rospy.logwarn("Timed out waiting for node interface valid...")
                return False
        return True

    def handle_config(self, msg):
        """
        config topic callback
        """
        if not self.config or _time_changed(self.config.time, msg.time):
            with self.config_mutex:
                self.config = msg
                self.config_changed()

    def handle_state(self, msg):
        """
        state topic callback
        """
        if not self.state or _time_changed(self.state.time, msg.time):
            with self.state_mutex:
                self.state = msg
                self.state_changed()

    def publish_command(self, op, args, timeout=2.0):
        """
        publish on the command topic
        return true if the command is acknowleged within the timeout
        """
        cmd_time = rospy.Time.now()
        self.cmd_times.append(cmd_time)
        self.cmd_times = self.cmd_times[-100:]
        cmd_msg = IOComponentCommand(
            time=cmd_time,
            op=op,
            args=json.dumps(args))
        rospy.loginfo("publish_command %s %s" % (cmd_msg.op, cmd_msg.args))
        if timeout != None:
            timeout_time = rospy.Time.now() + rospy.Duration(timeout)
            while not rospy.is_shutdown():
                self._command_pub.publish(cmd_msg)
                if self.is_valid():
                    if cmd_time in self.state.commands:
                        rospy.loginfo("command %s acknowleged" % (cmd_msg.op,))
                        return True
                rospy.sleep(0.1)
                if timeout_time < rospy.Time.now():
                    rospy.logwarn("Timed out waiting for command acknowlegment...")
                    break
            return False
        return True

class IONodeInterface(IOInterface):
    """
    IO Node interface to config, status and command topics
    """
    def __init__(self, node_name):
        super(IONodeInterface, self).__init__(
            'io/' + node_name,
            IONodeConfiguration,
            IONodeStatus)
        self.config = IONodeConfiguration()
        self.state = IONodeStatus()
        self.invalidate_config()
        self.invalidate_state()

    def get_device_status(self, device_name):
        """
        return the status for the given device, or none
        """
        with self.state_mutex:
            for device in self.state.devices:
                if device.name == device_name:
                    return device.status.tag
        return None


class IODeviceInterface(IOInterface):
    """
    IO Device interface to config, status and command topics
    """
    def __init__(self, node_name, dev_name):
        super(IODeviceInterface, self).__init__(
            'io/' + node_name + '/' + dev_name,
            IODeviceConfiguration,
            IODeviceStatus)
        self.config = IODeviceConfiguration()
        self.state = IODeviceStatus()
        self.invalidate_config()
        self.invalidate_state()

    def get_port(self, port_name):
        """
        return the status for the given port, or none
        """
        with self.state_mutex:
            for port in self.state.ports:
                if port.name == port_name:
                    return json.loads(port.data)
        return None

    def get_signal_value(self, signal_name):
        """
        return the status for the given signal, or none
        """
        with self.state_mutex:
            for signal in self.state.signals:
                if signal.name == signal_name:
                    return json.loads(signal.data)[0]
        return None

    def set_signal_value(self, signal_name, signal_value, signal_type=None, timeout=5.0):
        """
        set the value for the given signal
        return True if the signal ___ , False otherwise
        """
        if signal_type == None:
            s_type = self.get_signal_type(signal_name)
            if s_type == None:
                return False
        else:
            s_type = signal_type
        set_command = SetCommand().set_signal(signal_name, s_type, signal_value)
        self._publish_device_command(set_command, timeout=timeout)
        return True

    def get_signal_type(self, signal_name):
        """
        return the status for the given signal, or none
        """
        with self.state_mutex:
            for signal in self.state.signals:
                if signal.name == signal_name:
                    format = json.loads(signal.format)
                    if "type" in format:
                        return format["type"]
                    break
        return None

    def list_signals(self):
        """
        return a list of all signals
        """
        with self.state_mutex:
            return [signal.name for signal in self.state.signals]

    def list_ports(self):
        """
        return a list of all ports
        """
        with self.state_mutex:
            return [port.name for port in self.state.ports]

    def _publish_device_command(self, command, check_state=False, check_config=False, timeout=3.0):
        '''
        publish a command to the device
        '''
        if check_state:
            dev_interface.invalidate_state()
        if check_config:
            dev_interface.invalidate_config()
        self.publish_command(command.op, command.args, timeout=timeout)
        # make sure both state and config are valid:
        self.revalidate(timeout, invalidate_state=False, invalidate_config=False)
