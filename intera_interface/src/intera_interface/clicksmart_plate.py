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
from intera_core_msgs.msg import (
    IONodeStatus,
    IOComponentCommand
)
import intera_dataflow
from intera_io import (
    IODeviceInterface,
    IOCommand
)


class SimpleClickSmartGripper(object):
    """
    Bare bones Interface class for a ClickSmart gripper on the Intera Research Robot.

    This ClickSmart interface uses "End Effector (EE) Signal Types" to
    reference the Input/Output Signals configured on the ClickSmart device.
    EE Signal Types include types such as "grip", (is)"open", (is)"closed",
    (set)"object_kg". Each set of EE Signals is associated with a given
    endpoint_id, but this endpoint_id can be left out in most of the interfaces
    if a ClickSmart device only has a single endpoint configured.

    ClickSmart devices can be configured using the Intera Studio GUI. The
    configurations are stored on the ClickSmart device itself.
    """
    def __init__(self, ee_device_id, initialize=True):
        self.name = ee_device_id
        self.endpoint_map = None
        self.gripper_io = None
        self._node_state = None
        self._node_device_status = None

        self._node_command_pub = rospy.Publisher('io/end_effector/command', IOComponentCommand, queue_size=10)
        self._node_state_sub = rospy.Subscriber('io/end_effector/state', IONodeStatus, self._node_state_cb)

        # Wait for the gripper device status to be true
        intera_dataflow.wait_for(
            lambda: self._node_device_status is not None,
            timeout=5.0, raise_on_error=initialize,
            timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
        )

        self.gripper_io = IODeviceInterface("end_effector", self.name)
        self.gripper_io.config_changed.connect(self._load_endpoint_info)
        if self.gripper_io.is_config_valid():
            self._load_endpoint_info()

        if initialize and self.needs_init():
            self.initialize()

    def _node_state_cb(self, msg):
        # if we don't have a message yet, or if the timestamp is different: search for our device
        if not self._node_state or IODeviceInterface.time_changed(self._node_state.time, msg.time):
            self._node_state = msg
            if len(msg.devices) and msg.devices[0].name == self.name:
                self._node_device_status = msg.devices[0].status
            else:
                # device removed
                self._node_device_status = None

    def _load_endpoint_info(self):
        self._device_config = json.loads(self.config.device.config)
        self.endpoint_map = self._device_config['params']['endpoints']

    def is_ready(self):
        """
        Returns bool describing if the gripper is ready to control,
        (i.e. initialized ('activated'), not busy, no errors).

        @rtype: bool
        @return: True if in ready state, False otherwise
        """
        return (self._node_device_status and self._node_device_status.tag == 'ready'
            and self.gripper_io.is_valid())

    def needs_init(self):
        """
        Returns True if device is not initialized and can be initialized
        now ('activated').

        @rtype: bool
        @return: True if not initialized and can be, False otherwise
        """
        return (self._node_device_status and (self._node_device_status.tag == 'down'
            or self._node_device_status.tag == 'unready'))

    def initialize(self, timeout=5.0):
        """
        Activate ClickSmart - initializes device and signals and attaches
        the EE's URDF model to the internal RobotModel. Compensation
        of the EE's added mass may cause arm to move.

        @type timeout: float
        @param timeout: timeout in seconds
        """
        rospy.loginfo("Activating ClickSmart...")
        cmd = IOCommand('activate', {"devices": [self.name]})
        msg = cmd.as_msg()
        self._node_command_pub.publish(msg)
        if timeout:
            intera_dataflow.wait_for(
                lambda: self.is_ready(), timeout=timeout,
                timeout_msg=("Failed to initialize gripper.")
            )

    def get_ee_signal_value(self, ee_signal_type, endpoint_id=None):
        """
        Return current value of the given EE Signal on the ClickSmart.

        EE Signals are identified by "type" (such as "grip", (is)"open", etc.)
        and are configured per endpoint. If the ClickSmart has multiple endpoints,
        optionally specify the endpoint_id with which the signal is associated.

        @type ee_signal_type: str
        @param ee_signal_type: EE Signal Type of the signal to return; one of
            those listed in get_ee_signals().
        @type endpoint_id: str
        @param endpoint_id: endpoint_id associated with signal; (default: 1st endpoint found)

        @rtype: bool|float
        @return: value of signal
        """
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            return self.get_signal_value(endpoint_info[ee_signal_type])

    def set_ee_signal_value(self, ee_signal_type, value, endpoint_id=None, timeout=5.0):
        """
        Sets the value of the given EE Signal on the ClickSmart.

        EE Signals are identified by "type" (such as "grip", (is)"open", etc.)
        and are configured per endpoint. If the ClickSmart has multiple endpoints,
        optionally specify the endpoint_id with which the signal is associated.

        @type ee_signal_type: str
        @param ee_signal_type: EE Signal Type of the signal to return; one of
            those listed in get_ee_signals().
        @type value: bool|float
        @param value: new value to set the signal to
        @type endpoint_id: str
        @param endpoint_id: endpoint_id associated with signal; (default: 1st endpoint found)
        @type timeout: float
        @param timeout: timeout in seconds
        """
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            self.set_signal_value(endpoint_info[ee_signal_type], value)

    def get_ee_signals(self, endpoint_id=None):
        """
        Returns dict of EE Signals by EE Signal Type for given endpoint.

        Use the EE Signal Types (the keys) to specify signals in
        get_ee_signal_value() and set_ee_signal_value().

        EE Signals are identified by "type" (such as "grip", (is)"open", etc.)
        and are configured per endpoint. If the ClickSmart has multiple endpoints,
        optionally specify the endpoint_id with which the signal is associated.

        @type endpoint_id: str
        @param endpoint_id: endpoint_id associated with signal; (default: 1st endpoint found)
        @rtype: dict({str:str})
        @return: dict of EE Signal Types to signal_name
        """
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        exclude = ['endpoint_id', 'label', 'type', 'actuationTimeS']
        return {k: v for k,v in endpoint_info.iteritems() if k not in exclude}

    def get_all_ee_signals(self):
        """
        Returns dict of EE Signals by EE Signal Type for all endpoints,
        organized under endpoint_id.

        Use the EE Signal Types (the keys) to specify signals in
        get_ee_signal_value() and set_ee_signal_value().

        EE Signals are identified by "type" (such as "grip", (is)"open", etc.)
        and are configured per endpoint. If the ClickSmart has multiple endpoints,
        optionally specify the endpoint_id with which the signal is associated.

        @type endpoint_id: str
        @param endpoint_id: endpoint_id associated with signal; (default: 1st endpoint found)
        @rtype: dict({str:dict({str:str})})
        @return: dicts for each endpoint_id of EE Signal Types to signal_name
        """
        info = dict()
        for ept in self.list_endpoint_names():
            info[ept] = self.get_ee_signals(ept)
        return info

    def get_all_signals(self):
        """
        Returns generic info for all IO Signals on device, in a flat map keyed by signal_name.

        @rtype: dict
        @return: generic data-type, value, and IO Data for all signals
        """
        return self.gripper_io.signals

    def list_endpoint_names(self):
        """
        Returns list of endpoint_ids currently configured on this ClickSmart device.

        @rtype: list [str]
        @return: list of endpoint_ids in ClickSmart config
        """
        if self.endpoint_map:
            return self.endpoint_map.keys()
        else:
            return []

    def get_endpoint_info(self, endpoint_id=None):
        """
        Returns the endpoint_id and endpoint info for the default endpoint,
        or for the given endpoint_id if specified.

        Use to find the default endpoint_id by calling without args.
        Or lookup the current configuration info for a specific endpoint,
        including EE Signals, and Intera UI labels.

        @type endpoint_id: str
        @param endpoint_id: optional endpoint_id to lookup; default: looksup
            and returns info for the default endpoint_id of this ClickSmart
        @rtype: tuple [ str, dict]
        @return: (endpoint_id, endpoint_info) - the default endpoint_id (or the one specified)
            and signal/config info associated with endpoint
        """
        if self.endpoint_map is None or len(self.endpoint_map.keys()) <= 0:
            rospy.logerr('Cannot use endpoint signals without any endpoints!')
            return
        endpoint_id = self.endpoint_map.keys()[0] if endpoint_id is None else endpoint_id
        return (endpoint_id, self.endpoint_map[endpoint_id])

    def send_configuration(self, config, timeout=5.0):
        """
        Send a new JSON EndEffector configuration to the device.

        @type config: dict
        @param config: new json config to save on device
        @type timeout: float
        @param timeout: timeout in seconds - currently will wait for a minimum of
                        3 seconds before returning (to allow for reconfiguration)
        """
        rospy.loginfo("Reconfiguring ClickSmart...")

        cmd = IOCommand('reconfigure', {"devices": {self.name: config}, "write_config": True})
        msg = cmd.as_msg()
        self._node_command_pub.publish(msg)
        if timeout:
            # allow time for reconfiguration and storage write process
            # TODO: use cmd acknowlegment timestamp, not hard-coded delay
            delay = 3.0
            delay_time = rospy.Time.now() + rospy.Duration(delay)
            intera_dataflow.wait_for(
                lambda: (rospy.Time.now() > delay_time and
                    (self.is_ready() or self.needs_init())),
                timeout=max(timeout, delay),
                body=lambda: self._node_command_pub.publish(msg),
                timeout_msg=("Failed to reconfigure gripper.")
            )

    def __getattr__(self, name):
        """
        This is a proxy-pass through mechanism that lets you look up methods and variables
        on the underlying IODeviceInterface for the ClickSmart, as if they were on this class.
        """
        return getattr(self.gripper_io, name)
