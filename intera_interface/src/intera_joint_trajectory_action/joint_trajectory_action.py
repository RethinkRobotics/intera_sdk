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

"""
Intera RSDK Joint Trajectory Action Server
"""
import bisect
from copy import deepcopy
import math
import operator
import numpy as np

import bezier
import minjerk

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from std_msgs.msg import (
    UInt16,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import intera_control
import intera_dataflow
import intera_interface


class JointTrajectoryActionServer(object):
    def __init__(self, limb, reconfig_server, rate=100.0,
                 mode='position_w_id', interpolation='minjerk'):
        self._dyn = reconfig_server
        self._ns = 'robot/limb/' + limb
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._limb = intera_interface.Limb(limb, synchronous_pub=True)
        self._enable = intera_interface.RobotEnable()
        self._name = limb
        self._interpolation = interpolation
        self._cuff = intera_interface.Cuff(limb=limb)
        # Verify joint control mode
        self._mode = mode
        if (self._mode != 'position' and self._mode != 'position_w_id'
            and self._mode != 'velocity'):
            rospy.logerr("%s: Action Server Creation Failed - "
                         "Provided Invalid Joint Control Mode '%s' (Options: "
                         "'position_w_id', 'position', 'velocity')" %
                    (self._action_name, self._mode,))
            return
        self._server.start()
        self._alive = True
        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments, messages, and dynamic
        # reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()

        # Create our PID controllers
        self._pid = dict()
        for joint in self._limb.joint_names():
            self._pid[joint] = intera_control.PID()

        # Create our spline coefficients
        self._coeff = [None] * len(self._limb.joint_names())

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self._control_rate)

    def robot_is_enabled(self):
        return self._enable.state().enabled

    def clean_shutdown(self):
        self._alive = False
        self._limb.exit_control_mode()

    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = self._dyn.config['goal_time']
        # Stopped velocity tolerance - max velocity at end of execution
        self._stopped_velocity = self._dyn.config['stopped_velocity_tolerance']

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in self._limb.joint_names():
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
            # Path execution tolerance
            path_error = self._dyn.config[jnt + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config[jnt + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error

            # PID gains if executing using the velocity (integral) controller
            if self._mode == 'velocity':
                self._pid[jnt].set_kp(self._dyn.config[jnt + '_kp'])
                self._pid[jnt].set_ki(self._dyn.config[jnt + '_ki'])
                self._pid[jnt].set_kd(self._dyn.config[jnt + '_kd'])
                self._pid[jnt].initialize()

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_velocities(self, joint_names):
        return [self._limb.joint_velocity(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self, joint_names, joint_angles, start_time, dimensions_dict):
        if (self._cuff.cuff_button() or self._limb.has_collided() or
              not self.robot_is_enabled() or not self._alive):
            self._limb.exit_control_mode()
        elif self._mode == 'velocity':
            velocities = [0.0] * len(joint_names)
            cmd = dict(zip(joint_names, velocities))
            self._limb.set_joint_velocities(cmd)
        elif self._mode == 'position' or self._mode == 'position_w_id':
            pnt = JointTrajectoryPoint()
            pnt.positions = self._get_current_position(joint_names)
            if self._mode == 'position' and self._alive:
                self._limb.set_joint_positions(dict(zip(joint_names, pnt.positions)))
            if self._mode == 'position_w_id' and self._alive:
                # zero inverse dynamics feedforward command
                pnt.velocities = [0.0] * len(joint_names)
                pnt.accelerations = [0.0] * len(joint_names)
                self._limb.set_joint_trajectory(joint_names,
                                        pnt.positions,
                                        pnt.velocities,
                                        pnt.accelerations)

    def _command_joints(self, joint_names, point, start_time, dimensions_dict):
        if (self._limb.has_collided() or not self.robot_is_enabled()
             or not self._alive or self._cuff.cuff_button()):
           rospy.logerr("{0}: Robot arm in Error state. Stopping execution.".format(
                            self._action_name))
           self._limb.exit_control_mode()
           self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
           self._server.set_aborted(self._result)
           return False
        elif self._server.is_preempt_requested():
           rospy.logwarn("{0}: Trajectory execution Preempted. Stopping execution.".format(
                            self._action_name))
           self._limb.exit_control_mode()
           self._server.set_preempted()
           return False
        velocities = []
        deltas = self._get_current_error(joint_names, point.positions)
        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                  and self._path_thresh[delta[0]] >= 0.0)):
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._limb.exit_control_mode()
                return False
            if self._mode == 'velocity':
                velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        if self._mode == 'velocity':
            self._limb.set_joint_velocities(dict(zip(joint_names, velocities)))
        if self._mode == 'position':
            self._limb.set_joint_positions(dict(zip(joint_names, point.positions)))
        if self._mode == 'position_w_id':
            self._limb.set_joint_trajectory(joint_names,
                                            point.positions,
                                            point.velocities,
                                            point.accelerations)
        return True

    def _get_bezier_point(self, b_matrix, idx, t, cmd_time, dimensions_dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = b_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        if dimensions_dict['velocities']:
            pnt.velocities = [0.0] * num_joints
        if dimensions_dict['accelerations']:
            pnt.accelerations = [0.0] * num_joints
        for jnt in range(num_joints):
            b_point = bezier.bezier_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
            # Velocities at specified time
            if dimensions_dict['velocities']:
                pnt.velocities[jnt] = b_point[1]
            # Accelerations at specified time
            if dimensions_dict['accelerations']:
                pnt.accelerations[jnt] = b_point[-1]
        return pnt

    def _compute_bezier_coeff(self, joint_names, trajectory_points, dimensions_dict):
        # Compute Full Bezier Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_b_values = len(['b0', 'b1', 'b2', 'b3'])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_b_values))
        for jnt in xrange(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                if dimensions_dict['velocities']:
                    current_point.append(point.velocities[jnt])
                if dimensions_dict['accelerations']:
                    current_point.append(point.accelerations[jnt])
                traj_array[idx, :] = current_point
            d_pts = bezier.de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier.bezier_coefficients(traj_array, d_pts)
        return b_matrix

    def _get_minjerk_point(self, m_matrix, idx, t, cmd_time, dimensions_dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = m_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        if dimensions_dict['velocities']:
            pnt.velocities = [0.0] * num_joints
        if dimensions_dict['accelerations']:
            pnt.accelerations = [0.0] * num_joints
        for jnt in range(num_joints):
            m_point = minjerk.minjerk_point(m_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = m_point[0]
            # Velocities at specified time
            if dimensions_dict['velocities']:
                pnt.velocities[jnt] = m_point[1]
            # Accelerations at specified time
            if dimensions_dict['accelerations']:
                pnt.accelerations[jnt] = m_point[-1]
        return pnt

    def _compute_minjerk_coeff(self, joint_names, trajectory_points, point_duration, dimensions_dict):
        # Compute Full Minimum Jerk Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_m_values = len(['a0', 'a1', 'a2', 'a3', 'a4', 'a5', 'tm'])
        m_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_m_values))
        for jnt in xrange(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                if dimensions_dict['velocities']:
                    current_point.append(point.velocities[jnt])
                if dimensions_dict['accelerations']:
                    current_point.append(point.accelerations[jnt])
                traj_array[idx, :] = current_point
            m_matrix[jnt, :, :, :] = minjerk.minjerk_coefficients(traj_array, point_duration)
        return m_matrix

    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        velocity_flag = (len(trajectory_points[0].velocities) != 0 and
                         len(trajectory_points[-1].velocities) != 0)
        acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and
                             len(trajectory_points[-1].accelerations) != 0)
        return {'positions':position_flag,
                'velocities':velocity_flag,
                'accelerations':acceleration_flag}

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))
        for jnt_name, jnt_value in self._get_current_error(
                joint_names, trajectory_points[0].positions):
            if abs(self._path_thresh[jnt_name]) < abs(jnt_value):
                rospy.logerr(("{0}: Initial Trajectory point violates "
                             "threshold on joint {1} with delta {2} radians. "
                             "Aborting trajectory execution.").format(
                             self._action_name, jnt_name, jnt_value))
                self._server.set_aborted()
                return

        control_rate = rospy.Rate(self._control_rate)
        dimensions_dict = self._determine_dimensions(trajectory_points)

        # Force Velocites/Accelerations to zero at the final timestep
        # if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories,
        # and want continuous, non-zero velocities/accelerations between
        # trajectories
        if dimensions_dict['velocities']:
            trajectory_points[-1].velocities = [0.0] * len(joint_names)
        if dimensions_dict['accelerations']:
            trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        try:
            if self._interpolation == 'minjerk':
                # Compute Full MinJerk Curve Coefficients for all 7 joints
                point_duration = [pnt_times[i+1] - pnt_times[i] for i in range(len(pnt_times)-1)]
                m_matrix = self._compute_minjerk_coeff(joint_names,
                                                       trajectory_points,
                                                       point_duration,
                                                       dimensions_dict)
            else:
                # Compute Full Bezier Curve Coefficients for all 7 joints
                b_matrix = self._compute_bezier_coeff(joint_names,
                                                      trajectory_points,
                                                      dimensions_dict)
        except Exception as ex:
            rospy.logerr(("{0}: Failed to compute a Bezier trajectory for {1}"
                         " arm with error \"{2}: {3}\"").format(
                                                  self._action_name,
                                                  self._name,
                                                  type(ex).__name__, ex))
            self._server.set_aborted()
            return
        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        intera_dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown() and
               self.robot_is_enabled()):
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)
            #Calculate percentage of time passed in this interval
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0
                t = 0

            if self._interpolation == 'minjerk':
                point = self._get_minjerk_point(m_matrix, idx,
                                                t, cmd_time,
                                                dimensions_dict)
            else:
                point = self._get_bezier_point(b_matrix, idx,
                                               t, cmd_time,
                                               dimensions_dict)
            # Command Joint Position, Velocity, Acceleration
            command_executed = self._command_joints(joint_names, point, start_time, dimensions_dict)
            self._update_feedback(point, joint_names, now_from_start)
            if not command_executed:
                return
            # Sleep to make sure the publish is at a consistent time
            control_rate.sleep()

        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))

        def check_goal_state():
            for error in self._get_current_error(joint_names, last.positions):
                if (self._goal_error[error[0]] > 0
                        and self._goal_error[error[0]] < math.fabs(error[1])):
                    return error[0]
            if (self._stopped_velocity > 0.0 and
                max([abs(cur_vel) for cur_vel in self._get_current_velocities(joint_names)]) >
                    self._stopped_velocity):
                return False
            else:
                return True

        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown() and self.robot_is_enabled()):
            if not self._command_joints(joint_names, last, start_time, dimensions_dict):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        # Verify goal constraint
        result = check_goal_state()
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for %s arm" %
                          (self._action_name, self._name))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold for %s arm" %
                         (self._action_name, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s for %s arm" %
                         (self._action_name, result, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop(goal.trajectory.joint_names, end_angles, start_time, dimensions_dict)
