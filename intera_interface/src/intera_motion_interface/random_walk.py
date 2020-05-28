#! /usr/bin/env python

# Copyright (c) 2016-2018, Rethink Robotics Inc.
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


from copy import deepcopy
import random
import rospy

class RandomWalk(object):
    """
    This class is used to create a random walk in a bounded state space
    """

    def __init__(self):
        """
        Constructor - creates an empty random walk object.
        Be sure to set the lower limits, upper limits, and past point before
        calling getNextPoint().
        """

        self._lower_limit = None
        self._upper_limit = None
        self._last_point = None
        self._dimension = None
        self._boundary_padding = 0.0
        self._maximum_distance = 1.0

    def set_lower_limits(self, low):
        """
        @param low: [float]
        @return false if dimensions do not match
        @rtype: bool
        """
        if not self._check_dimension(low):
            return False
        self._lower_limit = deepcopy(low)
        return True

    def set_upper_limits(self, upp):
        """
        @param upp: [float]
        @return false if dimensions do not match
        @rtype: bool
        """
        if not self._check_dimension(upp):
            return False
        self._upper_limit = deepcopy(upp)
        return True

    def set_last_point(self, val):
        """
        Set the initial seed point
        @param low: val [float]
        @return false if dimensions do not match
        @rtype: bool
        """
        if not self._check_dimension(val):
            return False
        self._last_point = deepcopy(val)
        return True

    def set_boundary_padding(self, pad):
        """
        @param pad: bounday padding, must be [0,0.5)
        """
        if pad < 0.0:
            rospy.logwarn(' cannot set negative padding values')
            pad = 0.0
        elif pad > 0.499999:
            rospy.logwarn(' cannot set padding values greater than 0.5')
            pad = 0.499999
        self._boundary_padding = pad

    def set_maximum_distance(self, dist):
        """
        @param dist: maximum scaled step distance, must be (0,1]
        """
        if dist < 0.00001:
            rospy.logwarn(' distance must be positive!')
            dist = 0.00001
        elif dist > 1.0:
            rospy.logwarn(' cannot set max distance greater than 1.0')
            dist = 1.0
        self._maximum_distance = dist

    def get_last_point(self):
        """
        @return: the last point used
        @rtype: [float]
        """
        return deepcopy(self._last_point)

    def get_next_point(self):
        """
        @return: ext random point
        @rtype: [float]
        """
        if self._lower_limit is None:
            rospy.logerr('must define lower limits')
            return None
        elif self._upper_limit is None:
            rospy.logerr('must define upper limits')
            return None
        elif self._last_point is None:
            rospy.logerr('must define starting point')
            return None
        return self._get_next_point()

    def _get_next_point(self):
        point = []
        for i in range(0, self._dimension):
            low = self._lower_limit[i]
            upp = self._upper_limit[i]
            val = self._last_point[i]
            point.append(self._get_special_bounded_float(low, val, upp))
        self._last_point = point
        return deepcopy(point)

    def _get_special_bounded_float(self, low, val, upp):
        """
        @param low = hard lower bound on variable
        @param upp = hard upper bound on variable
        @param val = last sample value
        @param pad = normalized passing to apply to boundaries
        @param dist = normalized maximum distance from last value
        """
        pad = self._boundary_padding
        dist = self._maximum_distance
        r = upp - low
        lower = max(low + pad*r, val - dist*r)
        upper = min(upp - pad*r, val + dist*r)
        if lower > upper:
            print "No valid solution! Ignoring last sample value"
            lower = low + pad*r
            upper = upp - pad*r
        return lower + (upper-lower)*random.random()

    def _check_dimension(self, val):
        if self._dimension is None:
            self._dimension = len(val)
            return True
        elif self._dimension != len(val):
            rospy.logerr('vector length is not consistent!')
            return False
        return True
