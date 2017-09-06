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

import json
import rospy
from intera_core_msgs.msg import (
    IONodeConfiguration,
    IONodeStatus,
    IOComponentConfiguration,
    IOComponentCommand
)
import intera_dataflow
from intera_io import IODeviceInterface
from intera_io.io_command import IOCommand


class Gripper(object):
    """
    Interface class for a gripper on the Intera Research Robot.
    """
    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    MAX_FORCE = 10.0
    MIN_FORCE = 0.0
    MAX_VELOCITY = 3.0
    MIN_VELOCITY = 0.15

    def __init__(self, side="right", calibrate=True):
        """
        Constructor.

        @type side: str
        @param side: robot gripper name
        @type calibrate: bool
        @param calibrate: Attempts to calibrate the gripper when initializing class (defaults True)
        """
        self.devices = None
        self.name = '_'.join([side, 'gripper'])
        self.ee_config_sub = rospy.Subscriber('/io/end_effector/config', IONodeConfiguration, self._config_callback)
        # Wait for the gripper device status to be true
        intera_dataflow.wait_for(
            lambda: not self.devices is None, timeout=5.0,
            timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

        self.gripper_io = IODeviceInterface("end_effector", self.name)
        if self.has_error():
            self.reboot()
            calibrate = True
        if calibrate and not self.is_calibrated():
            self.calibrate()

    def _config_callback(self, msg):
        """
        config topic callback
        """
        if msg.devices != []:
            if str(msg.devices[0].name) == self.name:
                self.devices = msg

    def reboot(self):
        """
        Power cycle the gripper, removing calibration information.

        Basic call to the gripper reboot command. Waits for gripper to return
        ready state but does not clear errors that could occur during boot.

        @rtype: bool
        @return: True if successfully Rebooted, False otherwise
        """
        return self.gripper_io.set_signal_value("reboot", True)

    def stop(self):
        """
        Set the gripper to stop executing command at the current
        position, apply holding force.

        @rtype: bool
        @return: True if successfully Stopped, False otherwise
        """
        return self.gripper_io.set_signal_value("go", False)

    def start(self):
        """
        Set the gripper to start executing command at the current
        position, apply holding force.

        @rtype: bool
        @return: True if successfully Started, False otherwise
        """
        return self.gripper_io.set_signal_value("go", True)

    def open(self, position=MAX_POSITION):
        """
        Set the gripper position to open by providing opening position.
        @type: float
        @param: the postion of gripper in meters

        @rtype: bool
        @return: True if successfully set Position, False otherwise
        """
        return self.gripper_io.set_signal_value("position_m", position)

    def close(self, position=MIN_POSITION):
        """
        Set the gripper position to close by providing closing position.
        @type: float
        @param: the postion of gripper in meters

        @rtype: bool
        @return: True if successfully set Position, False otherwise
        """
        return self.gripper_io.set_signal_value("position_m", position)

    def has_error(self):
        """
        Returns a bool describing whether the gripper is in an error state.
        (True: Error detected, False: No errors)

        @rtype: bool
        @return: True if in error state, False otherwise
        """
        return self.gripper_io.get_signal_value("has_error")

    def is_ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is
        calibrated, not busy (as in resetting or rebooting), and
        not moving.

        @rtype: bool
        @return: True if in ready state, False otherwise
        """
        return (self.is_calibrated() and not self.has_error()
          and not self.is_moving())

    def is_moving(self):
        """
        Returns bool describing if the gripper is moving.

        @rtype: bool
        @return: True if in moving state, False otherwise
        """
        return self.gripper_io.get_signal_value("is_moving")

    def is_gripping(self):
        """
        Returns bool describing if the gripper is gripping.

        @rtype: bool
        @return: True if in gripping state, False otherwise
        """
        return self.gripper_io.get_signal_value("is_gripping")

    def is_calibrated(self):
        """
        Returns bool describing if the gripper is calibrated.

        @rtype: bool
        @return: True if successfully calibrated, False otherwise
        """
        return self.gripper_io.get_signal_value("is_calibrated")

    def calibrate(self):
        """
        Calibrate the gripper in order to set maximum and
        minimum travel distance.

        @rtype: bool
        @return: True if successfully calibrating, False otherwise
        """
        return self.gripper_io.set_signal_value("calibrate", True)

    def get_position(self):
        """
        Returns float describing the gripper position in meters.

        @rtype: float
        @return: Current Position value in Meters (m)
        """
        return self.gripper_io.get_signal_value("position_response_m")

    def set_position(self, position):
        """
        Set the position of gripper.
        @type: float
        @param: the postion of gripper in meters

        @rtype: bool
        @return: True if successfully set value, False otherwise
        """
        return self.gripper_io.set_signal_value("position_m", position)

    def set_velocity(self, speed):
        """
        Set the velocity at which the gripper position movement will execute.
        @type: float
        @param: the velocity of gripper in meters per second

        @rtype: float
        @return: Current Velocity value in Meters / second (m/s)
        """
        return self.gripper_io.set_signal_value("speed_mps", speed)

    def get_force(self):
        """
        Returns the force sensed by the gripper in estimated Newtons.

        @rtype: float
        @return: Current Force value in Newton-Meters (N-m)
        """
        return self.gripper_io.get_signal_value("force_response_n")

    def set_holding_force(self, holding_force):
        """
        Set holding force of successful gripper grasp.

        Set the force at which the gripper will continue applying after a
        position command has completed either from successfully achieving the
        commanded position, or by exceeding the moving force threshold.

        @type: float
        @param: the holding force of gripper in newton

        @rtype: bool
        @return: True if successfully set value, False otherwise
        """
        return self.gripper_io.set_signal_value("holding_force_n", holding_force)

    def set_object_weight(self, object_weight):
        """
        Set the weight of the object in kilograms.
        @type: float
        @param: the object weight in newton

        @rtype: bool
        @return: True if successfully set value, False otherwise
        """
        return self.gripper_io.set_signal_value("object_kg", object_weight)

    def set_dead_zone(self, dead_zone):
        """
        Set the gripper dead zone describing the position error threshold
        where a move will be considered successful.

        @type: float
        @param: the dead zone of gripper in meters

        @rtype: bool
        @return: True if successfully set value, False otherwise
        """
        return self.gripper_io.set_signal_value("dead_zone_m", dead_zone)


class SimpleIOGripper(object):
    """
    Bare bones Interface class for a gripper with a chip on the Intera Research Robot.
    """
    def __init__(self, ee_device_id, initialize=True):
        self.name = ee_device_id
        self.endpoint_map = None
        self._node_state = None
        self._node_device_status = None

        self._node_command_pub = rospy.Publisher('io/end_effector/command', IOComponentCommand, queue_size=10)
        self._node_state_sub = rospy.Subscriber('/io/end_effector/state', IONodeStatus, self._node_state_cb)

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
        if (not self._node_state or (
                self._node_state.time.secs != msg.time.secs or
                self._node_state.time.nsecs != msg.time.nsecs)):
            self._node_state = msg
            if len(msg.devices) and msg.devices[0].name == self.name:
                self._node_device_status = msg.devices[0].status
            else:
                # device removed
                self._node_device_status = None

    def _load_endpoint_info(self):
        self._device_config = json.loads(self.config.device.config)
        self.endpoint_map = self._device_config['params']['endpoints']

    def needs_init(self):
        return (self._node_device_status and (self._node_device_status.tag == 'down'
            or self._node_device_status.tag == 'unready'))

    def initialize(self, timeout=5.0):
        # activate tool
        cmd = IOCommand('activate', {"devices": [self.name]})
        msg = cmd.as_msg()
        self._node_command_pub.publish(msg)
        if timeout:
            intera_dataflow.wait_for(
                lambda: self.is_ready(), timeout=timeout,
                timeout_msg=("Failed to initialize gripper.")
            )

    def is_ready(self):
        return (self._node_device_status and self._node_device_status.tag == 'ready'
            and self.gripper_io.is_valid())

    def set_ee_signal_value(self, ee_signal_type, value, endpoint_id=None, timeout=5.0):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            return self.set_signal_value(endpoint_info[ee_signal_type], value)

    def get_ee_signal_value(self, ee_signal_type, endpoint_id=None):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            return self.get_signal_value(endpoint_info[ee_signal_type])

    def get_ee_signals(self, endpoint_id=None):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        exclude = ['endpoint_id', 'label', 'type', 'actuationTimeS']
        return {k: v for k,v in endpoint_info.iteritems() if k not in exclude}

    def get_all_ee_signals(self):
        info = dict()
        for ept in self.list_endpoint_names():
            info[ept] = self.get_ee_signals(ept)
        return info

    def get_all_signals(self):
        return self.gripper_io.signals

    def list_endpoint_names(self):
        if self.endpoint_map:
            return self.endpoint_map.keys()
        else:
            return []

    def get_endpoint_info(self, endpoint_id=None):
        if self.endpoint_map is None or len(self.endpoint_map.keys()) <= 0:
            rospy.logerr('Cannot use endpoint signals without any endpoints!')
            return
        endpoint_id = self.endpoint_map.keys()[0] if endpoint_id is None else endpoint_id
        return (endpoint_id, self.endpoint_map[endpoint_id])

    # proxy-pass through
    def __getattr__(self, name):
        return getattr(self.gripper_io, name)


class GripperFactory(object):

    def __init__(self, live=True):
        self.states = []
        self.configs = []
        # node interface
        self._node_state_sub = rospy.Subscriber('/io/end_effector/state', IONodeStatus, self._node_state_cb)
        self._node_config_sub = rospy.Subscriber('/io/end_effector/config', IONodeConfiguration, self._node_config_cb)

        if live:
            intera_dataflow.wait_for(
                lambda: not self.states is None,
                timeout=5.0, raise_on_error=False,
                timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

    def _node_state_cb(self, msg):
        self.node_state = msg
        if msg.devices:
            self.states = msg.devices

    def _node_config_cb(self, msg):
        self.node_config = msg
        if msg.devices:
            self.configs = msg.devices

    def lookupGripperClass(self, ee_type):
        EE_CLASS_MAP = dict({
            "SmartToolPlate": SimpleIOGripper,
            "ElectricParallelGripper": Gripper,
            "default": Gripper
        })

        return EE_CLASS_MAP.get(ee_type, None) or EE_CLASS_MAP["default"]

    def parseConfig(self, config):
        ee_config = config
        if type(ee_config) == IOComponentConfiguration:
            ee_config = ee_config.config
        if type(ee_config) == str:
            ee_config = json.loads(ee_config)
        if type(ee_config) == dict:
            return ee_config

    def getCurrentGripperInterface(self):

        if len(self.states) <= 0:
            # wait a second
            intera_dataflow.wait_for(
                lambda: (len(self.states) > 0 and len(self.configs) > 0),
                timeout=5.0, raise_on_error=False,
                timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

        if len(self.states) and len(self.configs):
            ee_state = self.states[0]
            ee_id = ee_state.name
            ee_config = None
            for conf in self.configs:
                if conf.name == ee_id:
                    ee_config = self.parseConfig(conf.config)
                    break

            needs_init = True if (ee_state.status.tag == 'down' or ee_state.status.tag == 'unready') else False

            gripper_class = self.lookupGripperClass(ee_config['props']['type'])

            gripper = None
            try:
                gripper = gripper_class(ee_id, needs_init)
            except:
                gripper = gripper_class(ee_id, False)
            return gripper
        else:
            return None
