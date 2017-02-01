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

import rospy
import intera_dataflow
from intera_io import IODeviceInterface
from intera_core_msgs.msg import IONodeConfiguration


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
