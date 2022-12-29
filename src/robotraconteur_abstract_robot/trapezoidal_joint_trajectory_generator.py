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
import RobotRaconteur as RR
from typing import NamedTuple

class JointTrajectoryLimits(NamedTuple):
    j_max: np.array
    a_max: np.array
    v_max: np.array
    x_min: np.array
    x_max: np.array

class JointTrajectoryPositionRequest(NamedTuple):
    current_position: np.array
    current_velocity: np.array
    desired_position: np.array
    desired_velocity: np.array
    max_velocity: np.array
    desired_time: float = None
    speed_ratio: float = 1.
    splice_time: float = None

class JointTrajectoryVelocityRequest(NamedTuple):
    current_position: np.array
    current_velocity: np.array
    desired_velocity: np.array
    timeout: float
    speed_ratio: float
    desired_time: float = None

class JointTrajectoryPositionCommand(NamedTuple):
    command_position: np.array
    command_velocity: np.array


class TrapezoidalJointTrajectoryGenerator:

    def __init__(self, joint_count, limits):
        self._joint_count=joint_count
        self._limits=limits

        self._speed_ratio = 1.0
        self._t_des = 0.0
        self._exec = None

    @property
    def t_des(self):
        return self._t_des

    @property
    def speed_ratio(self):
        return self._speed_ratio

    @property
    def t_final(self):
        if self._exec is not None:
            return self._exec.t_final
        else:
            return 0.

    @property
    def is_valid(self):
        return self._exec is not None

    @property
    def target_position(self):
        return self._exec.xf

    @property
    def target_velocity(self):
        return self._exec.v3

    def get_command(self, t):
        if self._exec is None:
            return False, None

        res, command_position, command_velocity = self._exec.calc_at_time(t)
        if not res:
            return False, None
        command = JointTrajectoryPositionCommand(
            command_position = command_position,
            command_velocity = command_velocity
        )

        return True, command

    def update_desired_position(self, request):
        assert request.desired_time is None, "desired time not supported"
        self._exec = TrapezoidalJointTrajectoryGeneratorCalc.initialize_pos_exec(self._joint_count, self._limits, request)
        return True

    def update_desired_velocity(self, request):
        self._exec = TrapezoidalJointTrajectoryGeneratorCalc.initialize_vel_exec(self._joint_count, self._limits, request)
        return True

class TrapezoidalJointTrajectoryGeneratorCalc:
    
    @classmethod
    def initialize_pos_exec(cls, joint_count, limits, request):
        assert request.desired_time is None

        a_max = np.copy(limits.a_max)
        v_max = np.copy(limits.v_max)

        if request.speed_ratio != 0.0:
            a_max *= request.speed_ratio
            v_max *= request.speed_ratio

            if request.max_velocity is not None:
                req_vel = request.max_velocity * request.speed_ratio
                assert np.all(req_vel <= v_max), "req_vel must be less than or equal to v_max"
                v_max = np.minimum(req_vel,v_max)

        else:            
            assert np.all(request.max_velocity <= v_max), "req_vel must be less than or equal to v_max"
            v_max = np.minimum(request.max_velocity,v_max)

        dx = request.desired_position - request.current_position

        t1 = np.zeros((joint_count,))
        t2 = np.zeros((joint_count,))
        t3 = np.zeros((joint_count,))

        v1 = np.zeros((joint_count,))
        a1 = np.zeros((joint_count,))
        a3 = np.zeros((joint_count,))

        for i in range(joint_count):
            if dx[i] == 0 and request.desired_velocity[i] == 0 and request.current_velocity[i] == 0:
                continue

            case2_success, v1[i], a1[i], a3[i], t1[i], t2[i], t3[i] = \
                cls.solve_case2(
                request.current_position[i], request.desired_position[i], request.current_velocity[i],
                request.desired_velocity[i], v_max[i], a_max[i]
            )

            if not case2_success:
                case3_success, a1[i], a3[i], t1[i], t3[i] = \
                    cls.solve_case3(
                    request.current_position[i], request.desired_position[i], request.current_velocity[i],
                    request.desired_velocity[i], a_max[i]
                )
                t2[i] = 0
                v1[i] = 0

                assert case3_success, "Invalid trajectory request"

        t1_4 = np.max(t1)
        t2_4 = np.max(t2)
        t3_4 = np.max(t3)

        a1_4 = np.zeros((joint_count,))
        a3_4 = np.zeros((joint_count,))
        v1_4 = np.zeros((joint_count,))

        for i in range(joint_count):
            v1_4[i], a1_4[i], a3_4[i] = cls.solve_case4(
                request.current_position[i], request.desired_position[i], request.current_velocity[i], 
                request.desired_velocity[i], t1_4, t2_4, t3_4)

        return TrapezoidalJointTrajectoryGeneratorExec(
            joint_count = joint_count,
            t1 = t1_4,
            t2 = t2_4,
            t3 = t3_4,
            x1 = request.current_position,
            v1 = request.current_velocity,
            v2 = v1_4,
            v3 = request.desired_velocity,
            a1 = a1_4,
            a3 = a3_4,
            xf = request.desired_position
        )

    @staticmethod
    def pos_1(a, v, x, t):
        return (0.5 * a * pow(t, 2) if a != 0 else 0.0) + v*t + x

    @staticmethod
    def vel_1(a, v, t):
        return (a *t if a != 0 else 0.0) + v

    @classmethod
    def pos(cls,a1, a3, x0, v0, t1, t2, t3):
        v1_p = cls.vel_1(a1, v0, t1)
        v3_p = cls.vel_1(a3, v1_p, t3)

        x1_p = cls.pos_1(a1, v0, x0, t1)
        x2_p = cls.pos_1(0, v1_p, x1_p, t2)
        x3_p = cls.pos_1(a3, v1_p, x2_p, t3)

        return x3_p, v3_p

    @classmethod
    def solve_case2_sub(cls, x0, xf, v0, v1, vf, a_max):
        t1 = 0
        a1 = 0
        if v1 != v0:
            a1 = a_max * np.sign(v1 - v0)
            t1 = (v1-v0) / a1
        
        t3 = 0
        a3 = 0
        if vf != v1:
            a3 = a_max * np.sign(vf-v1)
            t3 = (vf - v1) / a3

        xf_1, vf_1 = cls.pos(a1, a3, x0, v0, t1, 0.0, t3)

        dx2 = xf - xf_1

        t2 = dx2/v1

        return t2 >= 0, a1, a3, t1, t2, t3

    @classmethod
    def solve_case2(cls, x0, xf, v0, vf, v_max, a_max):
        case2_res, a1, a3, t1, t2, t3 = cls.solve_case2_sub(x0, xf, v0, v_max, vf, a_max)
        if case2_res:
            return True, v_max, a1, a3, t1, t2, t3

        case2_res, a1, a3, t1, t2, t3 = cls.solve_case2_sub(x0, xf, v0, -v_max, vf, a_max)
        if case2_res:
            return True, -v_max, a1, a3, t1, t2, t3

        return False, None, None, None, None, None, None


    @staticmethod
    def solve_case3_sub1(x0, xf, v0, vf, a1):
        return a1 * (xf - x0) + 0.5 * pow(v0, 2.0) + 0.5 * pow(vf, 2.0)

    @classmethod
    def solve_case3(cls, x0, xf, v0, vf, a_max):
        sub1 = cls.solve_case3_sub1(x0, xf, v0, vf, a_max)
        if sub1 >= 0:
            a1 = a_max
            a3 = -a_max
            t1 = (-v0 + np.sqrt(sub1)) / a1
            t3 = (a1*t1 + v0-vf) / a1
            if t1 > 0 and t3 > 0:
                return True, a1, a3, t1, t3
            t1 = (-v0 - np.sqrt(sub1))/ a1
            t3 = a1 * (a1 * t1 + v0 - vf) / a1
            if t1 > 0 and t3 > 0:
                return True, a1, a3, t1, t3

        sub1 = cls.solve_case3_sub1(x0, xf, v0, vf, -a_max)

        if sub1 >= 0:
            a1 = -a_max
            a3 = a_max
            t1 = (-v0 + np.sqrt(sub1)) / a1
            t3 = (a1 * t1 + v0 - vf) / a1
            if t1 > 0 and t3 > 0:
                return True, a1, a3, t1, t3

            t1 = (-v0 - np.sqrt(sub1)) / a1
            t3 = (a1 * t1 + v0 - vf) / a1
            if (t1 > 0 and t3 > 0):
                return True, a1, a3, t1, t3

        return False, None, None, None, None

    @classmethod
    def solve_case4(cls, x0, xf, v0, vf, t1, t2, t3):
        a1_den = t1 * (t1 + 2*t2 + t3)
        a1 = (-2 * t1 * v0 - 2 * t2 * v0 - t3 * v0 - t3 * vf - 2 * x0 + 2 * xf) / a1_den if a1_den !=0 else 0.0
        v1 = a1 * t1 + v0
        a3 = 0.0
        if t3 != 0:
            a3 = (-a1 * t1 - v0 + vf) / t3

        return v1, a1, a3
    
    @classmethod
    def initialize_vel_exec(cls, joint_count, limits, request):
        a_max = np.copy(limits.a_max)
        v_max = np.copy(limits.v_max)
        if request.speed_ratio != 0.0:
            a_max *= request.speed_ratio
            v_max *= request.speed_ratio

            req_vel = request.desired_velocity * request.speed_ratio
            assert np.all(req_vel <= v_max), "req_vel must be less than or equal to v_max"
            v_max = np.minimum(v_max, req_vel)

        else:
            assert np.all(request.max_velocity <= v_max), "req_vel must be less than or equal to v_max"
            v_max = np.minimum(request.max_velocity,v_max)

        t1 = np.zeros((joint_count,))
        t2 = np.zeros((joint_count,))
        t3 = np.zeros((joint_count,))

        v1 = np.zeros((joint_count,))
        a1 = np.zeros((joint_count,))
        a3 = np.zeros((joint_count,))

        for i in range(joint_count):
            if request.desired_velocity[i] == 0 and request.current_velocity[i] == 0:
                continue

            a_max[i], v1[i], a1[i], a3[i], t1[i], t2[i], t3[i] = cls.solve_case5(request.current_velocity[i], 
                request.desired_velocity[i], 0, request.timeout, a_max[i])

        t1_4 = np.max(t1)
        t2_4 = np.max(t2)
        t3_4 = np.max(t3)

        a1_4 = np.zeros((joint_count,))
        a3_4 = np.zeros((joint_count,))

        for i in range(joint_count):
            a1_4[i], a3_4[i] = cls.solve_case6(request.current_velocity[i], v1[i], 0, t1_4, t2_4, t3_4)

        ret = TrapezoidalJointTrajectoryGeneratorExec(
            joint_count = joint_count,
            t1 = t1_4,
            t2 = t2_4,
            t3 = t3_4,
            x1 = request.current_position,
            v1 = request.current_velocity,
            v2 = v1,
            v3 =np.zeros((joint_count,)),
            a1 = a1_4,
            a3 = a3_4,
            xf = None
        )

        return ret
            

    
    @staticmethod
    def solve_case5(v0, v1, vf, timeout, a_max):
        t1 = 0.0
        a1 = 0.0
        if (v1 != v0):        
            a1 = a_max * np.sign(v1 - v0)
            t1 = (v1 - v0) / a1
        
        if (t1 > timeout):        
            v1 = a1 * timeout
            t1 = timeout        

        t3 = 0
        a3 = 0
        if (vf != v1):        
            a3 = a_max * np.sign(vf - v1)
            t3 = (vf - v1) / a3        

        t2 = timeout - t1

        v1_res = v1

        return True, v1_res, a1, a3, t1, t2, t3

    @staticmethod
    def solve_case6(v0, v1, vf, t1, t2, t3):
        a1_den = t1
        a1 = (v1-v0) / a1_den  if (a1_den != 0) else 0.0
        a3 = 0
        if (t3 != 0):        
            a3 = (-a1 * t1 - v0 + vf) / t3

        return a1, a3
        


class TrapezoidalJointTrajectoryGeneratorExec:
    def __init__(self, joint_count, t1, t2, t3, x1, v1, v2, v3, a1, a3, xf):
        self.joint_count = joint_count
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.x1 = x1
        self.v1 = v1
        self.v2 = v2
        self.v3 = v3
        self.a1 = a1
        self.a3 = a3
        self.xf = xf
        self.t_final = t1 + t2 +t3
        self.x2 = None
        self.x3 = None

    @staticmethod
    def pos_1(a, v, x, t):
        return (0.5 * a * pow(t, 2) if a != 0 else 0.0) + v*t + x

    @staticmethod
    def vel_1(a, v, t):
        return (a *t if a != 0 else 0.0) + v

    @classmethod
    def pos(cls, n, a, v, x, t):
        o = np.zeros((n,))
        for i in range(n):
            if a is not None:
                o[i] = cls.pos_1(a[i], v[i], x[i], t)
            else:
                o[i] = cls.pos_1(0.0, v[i], x[i], t)
        
        return o

    @classmethod
    def vel(cls, n, a, v, t):
        o = np.zeros((n,))
        for i in range(n):
            if a is not None:
                o[i] = cls.vel_1(a[i], v[i], t)
            else:
                o[i] = cls.vel_1(0.0, v[i], t)

        return o

    def calc_at_time(self, t):
        if self.x2 is None:
            self.x2 = self.pos(self.joint_count, self.a1, self.v1, self.x1, self.t1)

        if self.x3 is None:
            self.x3 = self.pos(self.joint_count, None, self.v2, self.x2, self.t2)

        if self.xf is None:
            self.xf = self.pos(self.joint_count, self.a3, self.v2, self.x3, self.t3)

        if t < 0:
            return False, None, None

        if (t < self.t1):
            x = self.pos(self.joint_count, self.a1, self.v1, self.x1, t)
            v = self.vel(self.joint_count, self.a1, self.v1, t)
            return True, x, v

        if (t < self.t2 + self.t1):
            x = self.pos(self.joint_count, None, self.v2, self.x2, t - self.t1)
            v = self.vel(self.joint_count, None, self.v2, t - self.t1)
            return True, x, v

        if (t < self.t_final):
            x = self.pos(self.joint_count, self.a3, self.v2, self.x3, t - self.t1 -self.t2)
            v = self.vel(self.joint_count, self.a3, self.v2, t - self.t1 - self.t2)
            return True, x, v

        x = self.pos(self.joint_count, None, self.v3, self.xf, t - self.t_final)
        v = self.vel(self.joint_count, None, self.v3, t - self.t_final)
        return True, x, v
        