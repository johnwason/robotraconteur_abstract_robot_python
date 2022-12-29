# Copyright 2022 Wason Technology, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
from scipy.interpolate import CubicSpline
import RobotRaconteur as RR

class JointTrajectoryInterpolator:

    def __init__(self, info):
        self._joint_names = [j.joint_identifier.name for j in info.joint_info]
        self._joint_min = np.array([j.joint_limits.lower for j in info.joint_info])
        self._joint_max = np.array([j.joint_limits.upper for j in info.joint_info])
        self._joint_vel_max = np.array([j.joint_limits.velocity for j in info.joint_info])
        self._joint_splines = None
        self._max_t = 0
        self._joint_start = None
        self._joint_end = None
        self._waypoint_times = None

    def load_trajectory(self, traj, speed_ratio):

        if len(traj.joint_names) > 0:
            if traj.joint_names != self._joint_names:
                raise RR.InvalidArgumentException("Joint names in trajectory must match robot joint names")

        if traj.waypoints is None:
            raise RR.InvalidArgumentException("Waypoint list must not be null")

        if len(traj.waypoints) < 5:
            raise RR.InvalidArgumentException("Waypoint list must contain five or more waypoints")

        if traj.waypoints[0].time_from_start != 0:
            raise RR.InvalidArgumentException("Trajectory time_from_start must equal zero for first waypoint")

        n_waypoints = len(traj.waypoints)
        n_joints = len(self._joint_names)

        traj_t = np.zeros((n_waypoints,))
        traj_j = np.zeros((n_waypoints, n_joints))

        last_t = 0

        for i in range(n_waypoints):

            w = traj.waypoints[i]

            if (len(w.joint_position) != n_joints):
                raise RR.InvalidArgumentException(f"Waypoint {i} invalid joint array length")

            if len(w.joint_velocity) != n_joints and len(w.joint_velocity) != 0:
                raise RR.InvalidArgumentException(f"Waypoint {i} invalid joint velocity array length")

            if len(w.position_tolerance) != n_joints and len(w.position_tolerance) != 0:
                raise RR.InvalidArgumentException(f"Waypoint {i} invalid tolerance array length")

            if len(w.velocity_tolerance) != n_joints and len(w.velocity_tolerance) != 0:
                raise RR.InvalidArgumentException(f"Waypoint {i} invalid tolerance array length")

            if i > 0:
                if w.time_from_start/speed_ratio <= last_t:
                    raise RR.InvalidArgumentException(f"Waypoint {i} time_from_start must be increasing")

                if w.time_from_start/speed_ratio - last_t > 0.1:
                    raise RR.InvalidArgumentException("Waypoint {i} more than 100 ms from previous waypoint")

            if np.any(w.joint_position > self._joint_max) or np.any(w.joint_position < self._joint_min):
                raise RR.InvalidArgumentException(f"Waypoint {i} exceeds joint limits")

            if len(w.joint_velocity) > 0:
                if np.any(np.abs(w.joint_velocity*speed_ratio) > self._joint_vel_max):
                    raise RR.InvalidArgumentException(f"Waypoint {i} exceeds joint velocity limits")

            if i > 0:
                last_w = traj.waypoints[i-1]
                dt = w.time_from_start/speed_ratio - last_w.time_from_start/speed_ratio
                dj = np.abs(w.joint_position - last_w.joint_position)
                if np.any (dj/dt > self._joint_vel_max):
                    raise RR.InvalidArgumentException(f"Waypoint {i} exceeds joint velocity limits")

            traj_t[i] = w.time_from_start/speed_ratio
            traj_j[i,:] = w.joint_position
            last_t = w.time_from_start / speed_ratio 

        self._joint_splines = CubicSpline(traj_t, traj_j)
        self._max_t = last_t

        self._joint_start = traj.waypoints[0].joint_position
        self._joint_end = traj.waypoints[-1].joint_position
        self._waypoint_times = traj_t

    @property
    def max_time(self):
        return self._max_t

    def interpolate(self, time):
        if time <= 0:
            return True, self._joint_start, 0

        if time >= self._max_t:
            return True, self._joint_end, len(self._waypoint_times) -1

        joint_pos = self._joint_splines(time)

        a = np.where(self._waypoint_times <= time)[0]
        if len(a) > 0:
            current_waypoint = a[-1]
        else:
            current_waypoint = 0
        
        return True, joint_pos, current_waypoint