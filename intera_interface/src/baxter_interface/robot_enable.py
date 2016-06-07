# Copyright (c) 2013-2015, Rethink Robotics
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

import errno
import re
import sys

from threading import Lock

import rospy

from std_msgs.msg import (
    Bool,
    Empty,
)

import baxter_dataflow

from baxter_core_msgs.msg import (
    AssemblyState,
)
from baxter_interface import settings


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

        The compatibility of robot versions to SDK (baxter_interface)
        versions is defined in the L{baxter_interface.VERSIONS_SDK2ROBOT}.

        By default, the class does not check, but all examples do. The
        example behavior can be overridden by changing the value of
        L{baxter_interface.CHECK_VERSION} to False.
        """
        self._state = None
        state_topic = 'robot/state'
        self._state_sub = rospy.Subscriber(state_topic,
                                           AssemblyState,
                                           self._state_callback
                                           )
        if versioned and not self.version_check():
            sys.exit(1)

        baxter_dataflow.wait_for(
            lambda: not self._state is None,
            timeout=2.0,
            timeout_msg=("Failed to get robot state on %s" %
            (state_topic,)),
        )

    def _state_callback(self, msg):
        self._state = msg

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('robot/set_super_enable', Bool, 
                              queue_size=10)

        baxter_dataflow.wait_for(
            test=lambda: self._state.enabled == status,
            timeout=2.0 if status else 5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(status),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))

    def state(self):
        """
        Returns the last known robot state.
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
http://sdk.rethinkrobotics.com/wiki/RSDK_Shell#Initialize
"""
        is_reset = lambda: (self._state.enabled == False and
                            self._state.stopped == False and
                            self._state.error == False and
                            self._state.estop_button == 0 and
                            self._state.estop_source == 0)
        pub = rospy.Publisher('robot/set_super_reset', Empty, queue_size=10)

        if (self._state.stopped and
            self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED):
            rospy.logfatal(error_estop)
            raise IOError(errno.EREMOTEIO, "Failed to Reset: E-Stop Engaged")

        rospy.loginfo("Resetting robot...")
        try:
            baxter_dataflow.wait_for(
                test=is_reset,
                timeout=3.0,
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
        baxter_dataflow.wait_for(
            test=lambda: self._state.stopped == True,
            timeout=3.0,
            timeout_msg="Failed to stop the robot",
            body=pub.publish,
        )

    def version_check(self):
        """
        Verifies the version of the software running on the robot is
        compatible with this local version of the Baxter RSDK.

        Currently uses the variables in baxter_interface.settings and
        can be overridden for all default examples by setting CHECK_VERSION
        to False.
        """
        param_name = "rethink/software_version"
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
See: http://sdk.rethinkrobotics.com/wiki/Software_Update"""
                rospy.logerr(errstr_version, robot_version, sdk_version)
                return False
        return True
