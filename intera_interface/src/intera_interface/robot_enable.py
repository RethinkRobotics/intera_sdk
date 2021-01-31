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
import re
import sys

from threading import Lock

import rospy

from std_msgs.msg import (
    Bool,
    Empty,
)

import intera_dataflow

from intera_core_msgs.msg import (
    AssemblyState,
)
import settings


class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button
    """

    param_lock = Lock()

    def __init__(self, versioned=False):
        """
        Version checking capable constructor.

        @type versioned: bool
        @param versioned: True to check robot software version
        compatibility on initialization. False (default) to ignore.

        The compatibility of robot versions to SDK (intera_interface)
        versions is defined in the L{intera_interface.VERSIONS_SDK2ROBOT}.

        By default, the class does not check, but all examples do. The
        example behavior can be overridden by changing the value of
        L{intera_interface.CHECK_VERSION} to False.
        """
        self._state = None
        state_topic = 'robot/state'
        self._state_sub = rospy.Subscriber(state_topic,
                                           AssemblyState,
                                           self._state_callback
                                           )
        if versioned and not self.version_check():
            sys.exit(1)

        intera_dataflow.wait_for(
            lambda: not self._state is None,
            timeout=5.0,
            timeout_msg=("Failed to get robot state on %s" %
            (state_topic,)),
        )

    def _state_callback(self, msg):
        self._state = msg

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('robot/set_super_enable', Bool,
                              queue_size=10)

        intera_dataflow.wait_for(
            test=lambda: self._state.enabled == status,
            timeout=5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(status),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))

    def state(self):
        """
        Returns the last known robot state.

        @rtype: intera_core_msgs/AssemblyState
        @return: Returns the last received AssemblyState message
        """
        return self._state

    def enable(self):
        """
        Enable all joints
        """
        if self._state.stopped:
            rospy.loginfo("Robot Stopped: Attempting Reset...")
            self.reset()
        self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)

    def reset(self):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        error_not_stopped = """\
Robot is not in a Error State. Cannot perform Reset.
"""
        error_estop = """\
E-Stop is ASSERTED. Disengage E-Stop and then reset the robot.
"""
        error_nonfatal = """Non-fatal Robot Error on reset.
Robot reset cleared stopped state and robot can be enabled, but a non-fatal
error persists. Check diagnostics or rethink.log for more info.
"""
        error_env = """Failed to reset robot.
Please verify that the ROS_IP or ROS_HOSTNAME environment variables are set
and resolvable. For more information please visit:
http://sdk.rethinkrobotics.com/intera/SDK_Shell
"""


        is_reset = lambda: (self._state.stopped == False and
                            self._state.error == False and
                            self._state.estop_button == 0 and
                            self._state.estop_source == 0)
        pub = rospy.Publisher('robot/set_super_reset', Empty, queue_size=10)

        if (not self._state.stopped):
            rospy.logfatal(error_not_stopped)
            raise IOError(errno.EREMOTEIO, "Failed to Reset due to lack of Error State.")

        if (self._state.stopped and
              self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED):
            rospy.logfatal(error_estop)
            raise IOError(errno.EREMOTEIO, "Failed to Reset: E-Stop Engaged")

        rospy.loginfo("Resetting robot...")
        try:
            intera_dataflow.wait_for(
                test=is_reset,
                timeout=5.0,
                timeout_msg=error_env,
                body=pub.publish
            )
        except OSError, e:
            if e.errno == errno.ETIMEDOUT:
                if self._state.error == True and self._state.stopped == False:
                    rospy.logwarn(error_nonfatal)
                    return False
            raise

    def stop(self):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        pub = rospy.Publisher('robot/set_super_stop', Empty, queue_size=10)
        intera_dataflow.wait_for(
            test=lambda: self._state.stopped == True,
            timeout=5.0,
            timeout_msg="Failed to stop the robot",
            body=pub.publish,
        )

    def version_check(self):
        """
        Verifies the version of the software running on the robot is
        compatible with this local version of the Intera SDK.

        Currently uses the variables in intera_interface.settings and
        can be overridden for all default examples by setting CHECK_VERSION
        to False.

        @rtype: bool
        @return: Returns True if SDK version is compatible with robot Version, False otherwise
        """
        param_name = "/manifest/robot_software/version/HLR_VERSION_STRING"
        sdk_version = settings.SDK_VERSION

        # get local lock for rosparam threading bug
        with self.__class__.param_lock:
            robot_version = rospy.get_param(param_name, None)
        if not robot_version:
            rospy.logwarn("RobotEnable: Failed to retrieve robot version "
                          "from rosparam: %s\n"
                          "Verify robot state and connectivity "
                          "(i.e. ROS_MASTER_URI)", param_name)
            return False
        else:
            # parse out first 3 digits of robot version tag
            pattern = ("^([0-9]+)\.([0-9]+)\.([0-9]+)")
            match = re.search(pattern, robot_version)
            if not match:
                rospy.logwarn("RobotEnable: Invalid robot version: %s",
                              robot_version)
                return False
            robot_version = match.string[match.start(1):match.end(3)]
            if robot_version not in settings.VERSIONS_SDK2ROBOT[sdk_version]:
                errstr_version = """RobotEnable: Software Version Mismatch.
Robot Software version (%s) does not match local SDK version (%s). Please
Update your Robot Software. \
See: http://sdk.rethinkrobotics.com/intera/Software_Update"""
                rospy.logerr(errstr_version, robot_version, sdk_version)
                return False
        return True
