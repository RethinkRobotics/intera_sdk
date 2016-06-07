#!/usr/bin/env python

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
"""
Baxter RSDK Joint Trajectory Example: file playback
"""

import argparse
import operator
import sys
import threading

from bisect import bisect
from copy import copy
from os import path

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self):
        #create our action server clients
        self._left_client = actionlib.SimpleActionClient(
            'robot/limb/left/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #verify joint trajectory action servers are available
        l_server_up = self._left_client.wait_for_server(rospy.Duration(10.0))
        r_server_up = self._right_client.wait_for_server(rospy.Duration(10.0))
        if not l_server_up or not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        #create our goal request
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')

        #gripper interface - for gripper command playback
        self._l_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self._r_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        #flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        #reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()

        # Verify Grippers Have No Errors and are Calibrated
        if self._l_gripper.error():
            self._l_gripper.reset()
        if self._r_gripper.error():
            self._r_gripper.reset()
        if (not self._l_gripper.calibrated() and
            self._l_gripper.type() != 'custom'):
            self._l_gripper.calibrate()
        if (not self._r_gripper.calibrated() and
            self._r_gripper.type() != 'custom'):
            self._r_gripper.calibrate()

        #gripper goal trajectories
        self._l_grip = FollowJointTrajectoryGoal()
        self._r_grip = FollowJointTrajectoryGoal()

        # Timing offset to prevent gripper playback before trajectory has started
        self._slow_move_offset = 0.0
        self._trajectory_start_offset = rospy.Duration(0.0)
        self._trajectory_actual_offset = rospy.Duration(0.0)

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #gripper control rate
        self._gripper_rate = 20.0  # Hz

    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        l_cmd = self._l_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        now_from_start = rospy.get_time() - start_time
        while(now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown()):
            idx = bisect(pnt_times, now_from_start) - 1
            if self._r_gripper.type() != 'custom':
                self._r_gripper.command_position(r_cmd[idx].positions[0])
            if self._l_gripper.type() != 'custom':
                self._l_gripper.command_position(l_cmd[idx].positions[0])
            rate.sleep()
            now_from_start = rospy.get_time() - start_time

    def _clean_line(self, line, joint_names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param joint_names: joint name keys

        @return command: returns dictionary {joint: value} of valid commands
        @return line: returns list of current line values stripped of commas
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        #convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(joint_names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        return (command, line,)

    def _add_point(self, positions, side, time):
        """
        Appends trajectory with new point

        @param positions: joint positions
        @param side: limb to command point
        @param time: time from start for point in seconds
        """
        #creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'left':
            self._l_goal.trajectory.points.append(point)
        elif side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'left_gripper':
            self._l_grip.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)

    def parse_file(self, filename):
        """
        Parses input file into FollowJointTrajectoryGoal format

        @param filename: input filename
        """
        #open recorded file
        with open(filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in joint_names:
            if 'left' == name[:-3]:
                self._l_goal.trajectory.joint_names.append(name)
            elif 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'left' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        for idx, values in enumerate(lines[1:]):
            #clean each line of file
            cmd, values = self._clean_line(values, joint_names)
            #find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording
                cur_cmd = [self._l_arm.joint_angle(jnt) for jnt in self._l_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'left', 0.0)
                cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(start_offset + values[0])
            #add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'left', values[0] + start_offset)
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values[0] + start_offset)
            cur_cmd = [cmd['left_gripper']]
            self._add_point(cur_cmd, 'left_gripper', values[0] + start_offset)
            cur_cmd = [cmd['right_gripper']]
            self._add_point(cur_cmd, 'right_gripper', values[0] + start_offset)

    def _feedback(self, data):
        # Test to see if the actual playback time has exceeded
        # the move-to-start-pose timing offset
        if (not self._get_trajectory_flag() and
              data.actual.time_from_start >= self._trajectory_start_offset):
            self._set_trajectory_flag(value=True)
            self._trajectory_actual_offset = data.actual.time_from_start

    def _set_trajectory_flag(self, value=False):
        with self._lock:
            # Assign a value to the flag
            self._arm_trajectory_started = value

    def _get_trajectory_flag(self):
        temp_flag = False
        with self._lock:
            # Copy to external variable
            temp_flag = self._arm_trajectory_started
        return temp_flag

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        self._left_client.send_goal(self._l_goal, feedback_cb=self._feedback)
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag():
            rospy.sleep(0.05)
        self._execute_gripper_commands()

    def stop(self):
        """
        Preempts trajectory execution by sending cancel goals
        """
        if (self._left_client.gh is not None and
            self._left_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._left_client.cancel_goal()

        if (self._right_client.gh is not None and
            self._right_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._right_client.cancel_goal()

        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def wait(self):
        """
        Waits for and verifies trajectory execution result
        """
        #create a timeout for our trajectory execution
        #total time trajectory expected for trajectory execution plus a buffer
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)

        #verify result
        if all([l_finish, r_finish, l_result, r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False


def main():
    """RSDK Joint Trajectory Example: File Playback

    Plays back joint positions honoring timestamps recorded
    via the joint_recorder example.

    Run the joint_recorder.py example first to create a recording
    file for use with this example. Then make sure to start the
    joint_trajectory_action_server before running this example.

    This example will use the joint trajectory action server
    with velocity control to follow the positions and times of
    the recorded motion, accurately replicating movement speed
    necessary to hit each trajectory point on time.
    """
    epilog = """
Related examples:
  joint_recorder.py; joint_position_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of playback loops. 0=infinite.'
    )
    # remove ROS args and filename (sys.arv[0]) for argparse
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = Trajectory()
    traj.parse_file(path.expanduser(args.file))
    #for safe interrupt handling
    rospy.on_shutdown(traj.stop)
    result = True
    loop_cnt = 1
    loopstr = str(args.loops)
    if args.loops == 0:
        args.loops = float('inf')
        loopstr = "forever"
    while (result == True and loop_cnt <= args.loops
           and not rospy.is_shutdown()):
        print("Playback loop %d of %s" % (loop_cnt, loopstr,))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1
    print("Exiting - File Playback Complete")

if __name__ == "__main__":
    main()
