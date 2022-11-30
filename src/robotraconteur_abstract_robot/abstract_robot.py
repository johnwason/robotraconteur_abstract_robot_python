from enum import Enum
import traceback
from turtle import down
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
from RobotRaconteurCompanion.Util.RobotUtil import RobotUtil
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
import time
import threading
import numpy as np
from abc import ABC, abstractmethod

from .joint_trajectory_interpolator import JointTrajectoryInterpolator
from .trapezoidal_joint_trajectory_generator import JointTrajectoryLimits, JointTrajectoryPositionRequest, \
    JointTrajectoryVelocityRequest, JointTrajectoryPositionCommand, TrapezoidalJointTrajectoryGenerator

class AbstractRobot(ABC):
    def __init__(self, robot_info, default_joint_count, node = None ):
        super().__init__()

        if node is None:
            self._node = RRN
        else:
            self._node = node

        self._robot_info = robot_info
        if robot_info.joint_info is not None:
            j_names = []
            for j_info in robot_info.joint_info:
                j_names.append(j_info.joint_identifier.name)
            self._joint_names = j_names
        else:
            assert default_joint_count > 0, "Joints must be specified in RobotInfo structure"
            self._joint_names = [f"joint_{x}" for x in range(default_joint_count)]
        
        self._joint_count = len(self._joint_names)

        self._robot_uuid = robot_info.device_info.device.uuid

        self._robot_caps = robot_info.robot_capabilities

        self._robot_util = RobotUtil(self._node)
        self._datetime_util = DateTimeUtil(self._node)
        self._geometry_util = GeometryUtil(self._node)
        self._sensor_data_util = SensorDataUtil(self._node)

        self._pose_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.Pose")
        self._spatial_velocity_dtype = self._node.GetNamedArrayDType("com.robotraconteur.geometry.SpatialVelocity")
        self._robot_state_type = self._node.GetStructureType("com.robotraconteur.robotics.robot.RobotState")
        self._advanced_robot_state_type = self._node.GetStructureType("com.robotraconteur.robotics.robot.AdvancedRobotState")
        self._robot_state_sensor_data_type = self._node.GetStructureType("com.robotraconteur.robotics.robot.RobotStateSensorData")
        self._robot_joint_command_type = self._node.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand")
        self._isoch_info_type = self._node.GetStructureType("com.robotraconteur.device.isoch.IsochInfo")

        self._robot_consts = self._node.GetConstants("com.robotraconteur.robotics.robot")
        self._robot_capabilities = self._robot_consts["RobotCapabilities"]
        self._robot_command_mode = self._robot_consts["RobotCommandMode"]
        self._robot_operational_mode = self._robot_consts["RobotOperationalMode"]
        self._robot_controller_state = self._robot_consts["RobotControllerState"]
        self._robot_state_flags = self._robot_consts["RobotStateFlags"]

        self._joint_consts = self._node.GetConstants("com.robotraconteur.robotics.joints")
        self._joint_position_units = self._joint_consts["JointPositionUnits"]
        self._joint_effort_units = self._joint_consts["JointEffortUnits"]

        self._uses_homing = (self._robot_caps & self._robot_capabilities["homing_command"]) != 0
        self._has_position_command = (self._robot_caps & self._robot_capabilities["position_command"]) != 0
        self._has_velocity_command = (self._robot_caps & self._robot_capabilities["velocity_command"]) != 0
        self._has_jog_command = (self._robot_caps & self._robot_capabilities["jog_command"]) != 0

        try:
            self._rox_robots = []
            for chain_i in range(len(self._robot_info.chains)):
                self._rox_robots.append(self._robot_util.robot_info_to_rox_robot(self._robot_info,chain_i))
        except:
            traceback.print_exc()
            raise ValueError("invalid robot_info, could not populate GeneralRoboticsToolbox.Robot")

        self._current_tool = [None]*len(self._robot_info.chains)
        self._current_payload = [None]*len(self._robot_info.chains)

        for i in range(len(self._robot_info.chains)):
            if self._robot_info.chains[i].current_tool is not None:
                self._current_tool[i] = self._robot_info.chains[i].current_tool

            if self._robot_info.chains[i].current_payload is not None:
                self._current_payload[i] = self._robot_info.chains[i].current_payload

        for i in range(self._joint_count):
            limits = robot_info.joint_info[i].joint_limits
            assert limits.velocity > 0, f"Invalid joint velocity for joint {i}"
            if limits.reduced_velocity <= 0:
                limits.reduced_velocity = limits.velocity

            assert limits.acceleration > 0, f"Invalid joint acceleration for joint {i}"
            if limits.reduced_acceleration <= 0:
                limits.reduced_acceleration = limits.acceleration

        self._keep_going = False

        self._stopwatch_epoch = None
        self._stopwatch_start = None

        self._loop_thread = None
        self._update_period = 0.01

        self._wait_event = threading.Event()

        self._last_robot_state = 0
        self._last_joint_state = 0
        self._last_endpoint_state = 0

        self._state_seqno = 0

        self._speed_ratio = 1.0

        self._jog_joint_limit = np.deg2rad(1000.)
        self._trajectory_error_tol = np.deg2rad(5.)

        self._command_mode = self._robot_command_mode["halt"]
        self._operational_mode = self._robot_operational_mode["manual_reduced_speed"]
        self._controller_state = self._robot_operational_mode["undefined"]

        self._joint_position = np.zeros((0,))
        self._joint_velocity = np.zeros((0,))
        self._joint_effort = np.zeros((0,))

        self._position_command = None
        self._velocity_command = None

        self._endpoint_pose = []
        self._endpoint_vel = []

        self._homed = False
        self._ready = False
        self._enabled = False
        self._stopped = False
        self._error = False
        self._estop_source = 0

        self._communication_failure = True
        self._communication_timeout = 0.25

        self._broadcast_downsampler = None

        self._wire_position_command_sent = False
        self._wire_velocity_command_sent = False
        self._wire_position_command_last_seqno = 0
        self._wire_velocity_command_last_seqno = 0
        self._wire_position_command_last_ep = 0
        self._wire_velocity_command_last_ep = 0

        self._trajectory_valid = False
        self._trajectory_current_time = 0
        self._trajectory_max_time = 0
        self._trajectory_waypoint = 0

        self._lock = threading.Lock()

        self._wires_ready = False

        self._active_trajectory = None
        self._queued_trajectories = []

        self._jog_start_time = 0.
        self._jog_trajectory_generator = None
        self._jog_completion_handler = None

        self._joint_position_command = None
        self._joint_velocity_command = None

        self._config_seqno = 1

        self._base_set_operational_mode = True
        self._base_set_controller_state = True

    def RRServiceObjectInit(self, context, service_path):
        self.robot_state_sensor_data.MaxBacklog = 3

        self._broadcast_downsampler = RR.BroadcastDownsampler(context, 0)
        self._broadcast_downsampler.AddPipeBroadcaster(self.robot_state_sensor_data)
        self._broadcast_downsampler.AddWireBroadcaster(self.robot_state)
        self._broadcast_downsampler.AddWireBroadcaster(self.advanced_robot_state)
        self._broadcast_downsampler.AddWireBroadcaster(self.device_clock_now)

        self._wires_ready = True

    def _perf_counter(self):
        return time.perf_counter()

    def _stopwatch_ellapsed_s(self):
        return self._perf_counter() - self._stopwatch_start

    def _start_robot(self):

        self._stopwatch_epoch = self._datetime_util.TimeSpec2Now()
        self._stopwatch_start = self._perf_counter()

        self._keep_going = True
        self._loop_thread = threading.Thread(target = self._loop_thread_func)
        self._loop_thread.daemon = True
        self._loop_thread.start()

    def _stop_robot(self):
        self._keep_going = False
        self._loop_thread.join()

    def _loop_thread_func(self):

        next_wait = self._stopwatch_ellapsed_s()
        now = next_wait

        while self._keep_going:

            self._run_timestep(now)

            now = self._stopwatch_ellapsed_s()

            while True:
                next_wait += self._update_period
                if next_wait > now:
                    break

            while True:
                now = self._stopwatch_ellapsed_s()
                if now >= next_wait:
                    break
                self._wait_event.wait(timeout=(next_wait-now))

            
    def _close(self):
        self._keep_going = False
        try:
            self._loop_thread.join(timeout=1)
        except:
            pass

    

    def _run_timestep(self, now):
        res = False
        joint_pos_cmd = None
        joint_vel_cmd = None

        rr_robot_state = None
        rr_advanced_robot_state = None
        rr_state_sensor_data = None
        downsampler_step = None

        with self._lock:
            if self._wires_ready:
                downsampler_step = RR.BroadcastDownsamplerStep(self._broadcast_downsampler)

            self._state_seqno += 1

            res = self._verify_communication(now)
            res = res and self._verify_robot_state(now)
            res_fill, joint_pos_cmd, joint_vel_cmd = self._fill_robot_command(now)
            res = res and res_fill

            rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data = self._fill_states(now)

        if res:
            self._send_robot_command(now, joint_pos_cmd, joint_vel_cmd)

        if downsampler_step:
            with downsampler_step:
                self._send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data)

    def _fill_state_flags(self, now):

        f = 0
        if self._communication_failure:
            f |= self._robot_state_flags["communication_failure"]
            return f

        if self._error:
            f |= self._robot_state_flags["error"]

        if self._stopped:
            f |= self._robot_state_flags["estop"]

            if self._estop_source == 0:
                pass
            elif self._estop_source == 1:
                f |= self._robot_state_flags["estop_button1"]
            elif self._estop_source == 2:
                f |= self._robot_state_flags["estop_other"]
            elif self._estop_source == 3:
                f |= self._robot_state_flags["estop_fault"]
            elif self._estop_source == 4:
                f |= self._robot_state_flags["estop_internal"]
        
        if self._enabled:
            f |= self._robot_state_flags["enabled"]

        if self._ready:
            f |= self._robot_state_flags["ready"]

        if self._uses_homing:
            if self._homed:
                f |= self._robot_state_flags["homed"]
            else:
                f |= self._robot_state_flags["homing_required"]

        if self._wire_position_command_sent:
            f |= self._robot_state_flags["valid_position_command"]

        if self._wire_velocity_command_sent:
            f |= self._robot_state_flags["valid_velocity_command"]

        if self._trajectory_valid:
            f |= self._robot_state_flags["trajectory_running"]

        return f

    def _calc_endpoint_pose(self, chain):

        # CALL LOCKED!
        if self._current_tool[chain] is None:
            return self._endpoint_pose[chain]

        endpoint_transform = self._geometry_util.pose_to_rox_transform(self._endpoint_pose[chain])
        tool_transform = self._geometry_util.transform_to_rox_transform(self._current_tool[chain].tcp)
        res = endpoint_transform * tool_transform
        return self._geometry_util.rox_transform_to_pose(res)

    def _calc_endpoint_poses(self):

        if self._endpoint_pose is None:
            return np.zeros((0,), dtype=self._pose_dtype)
        n = len(self._endpoint_pose)
        o = np.zeros((n,), dtype=self._pose_dtype)
        for i in range(n):
            o[i] = self._calc_endpoint_pose(i)
        return o

    def _calc_endpoint_vel(self, chain):

        # CALL LOCKED!

        if self._current_tool[chain] is None:
            return self._endpoint_vel[chain]

        endpoint_vel = self._geometry_util.spatial_velocity_to_array(self._endpoint_vel).flatten()
        endpoint_vel_ang = endpoint_vel[0:3]
        endpoint_vel_lin = endpoint_vel[3:7]
        current_tool_p = self._geometry_util.point_to_xyz(self._current_tool[chain].tcp["translation"])

        endpoint_transform = self._geometry_util.pose_to_rox_transform(self._endpoint_pose[chain])

        
        vel = endpoint_vel_lin + np.cross(endpoint_vel_ang, np.matmul(endpoint_transform.R, current_tool_p))

        return self._geometry_util.array_to_spatial_acceleration(np.concatenate((endpoint_vel_ang, vel)))

    def _calc_endpoint_vels(self):

        if self._endpoint_vel is None:
            return np.zeros((0,),dtype=self._spatial_velocity_dtype)

        n = len(self._endpoint_vel)
        o = np.zeros((n,),dtype=self._spatial_velocity_dtype)
        for i in range(n):
            o[i] = self._calc_endpoint_vel(i)
        
        return o

    def _fill_states(self, now):
        ts = self._datetime_util.TimeSpec3Now()

        rob_state = self._robot_state_type()               
        rob_state.ts = ts
        rob_state.seqno = self._state_seqno
        rob_state.command_mode = self._command_mode
        rob_state.operational_mode = self._operational_mode
        rob_state.controller_state = self._controller_state

        flags = self._fill_state_flags(now)

        rob_state.robot_state_flags = flags

        rob_state.joint_position = np.copy(self._joint_position)
        rob_state.joint_velocity = np.copy(self._joint_velocity)
        rob_state.joint_effort = np.copy(self._joint_effort)
        rob_state.joint_position_command = self._joint_position_command if self._joint_position_command is not None \
                else np.zeros((0,))
        rob_state.joint_velocity_command = self._joint_velocity_command if self._joint_velocity_command is not None \
                else np.zeros((0,))
        rob_state.kin_chain_tcp = self._calc_endpoint_poses()
        rob_state.kin_chain_tcp_vel = self._calc_endpoint_vels()
        rob_state.trajectory_running = self._trajectory_valid

        a_rob_state = self._advanced_robot_state_type()
        a_rob_state.ts = ts
        a_rob_state.seqno = rob_state.seqno
        a_rob_state.command_mode = rob_state.command_mode
        a_rob_state.operational_mode = rob_state.operational_mode
        a_rob_state.controller_state = rob_state.controller_state
        a_rob_state.robot_state_flags = rob_state.robot_state_flags
        a_rob_state.joint_position = rob_state.joint_position
        a_rob_state.joint_velocity = rob_state.joint_velocity
        a_rob_state.joint_effort = rob_state.joint_effort
        a_rob_state.joint_position_command = rob_state.joint_position_command
        a_rob_state.joint_velocity_command = rob_state.joint_velocity_command
        a_rob_state.kin_chain_tcp = rob_state.kin_chain_tcp
        a_rob_state.kin_chain_tcp_vel = rob_state.kin_chain_tcp_vel
        a_rob_state.trajectory_running = rob_state.trajectory_running
        a_rob_state.joint_position_units = [self._joint_position_units["radian"]]*self._joint_count
        a_rob_state.joint_effort_units = [self._joint_effort_units["newton_meter"]]*self._joint_count
        a_rob_state.trajectory_running = self._trajectory_valid
        a_rob_state.trajectory_time = self._trajectory_current_time
        a_rob_state.trajectory_max_time = self._trajectory_max_time
        a_rob_state.trajectory_current_waypoint = self._trajectory_waypoint
        a_rob_state.config_seqno = self._config_seqno

        sensor_data_header = self._sensor_data_util.FillSensorDataHeader(self._robot_info.device_info, self._state_seqno)

        sensor_data = self._robot_state_sensor_data_type()
        sensor_data.data_header = sensor_data_header
        sensor_data.robot_state = a_rob_state

        return rob_state, a_rob_state, sensor_data


    def _send_states(self, now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data):
        
        if not self._wires_ready:
            return
             
        self.robot_state.OutValue = rr_robot_state
        self.advanced_robot_state.OutValue = rr_advanced_robot_state
        self.robot_state_sensor_data.AsyncSendPacket(rr_state_sensor_data, lambda: None)
        self.device_clock_now.OutValue = self._datetime_util.FillDeviceTime(self._robot_info.device_info, self._state_seqno)

    @abstractmethod
    def _send_disable(self, handler):
        pass

    def async_disable(self, handler):
        self._send_disable(handler)

    @abstractmethod
    def _send_enable(self, handler):
        pass

    def async_enable(self, handler):
        self._send_enable(handler)

    @abstractmethod
    def _send_reset_errors(self, handler):
        pass

    def async_reset_errors(self, handler):
        self._send_reset_errors(handler)

    def _verify_communication(self, now):
        if (now - self._last_joint_state) > self._communication_timeout \
            or (now - self._last_robot_state) > self._communication_timeout \
            or (now - self._last_endpoint_state) > self._communication_timeout :

            self._communication_failure = True

            self._command_mode = self._robot_command_mode["invalid_state"]
            if self._base_set_operational_mode:
                self._operational_mode = self._robot_operational_mode["undefined"]
                self._controller_state = self._robot_controller_state["undefined"]

            self._joint_position = np.zeros((0,))
            self._joint_velocity = np.zeros((0,))
            self._joint_effort = np.zeros((0,))

            self._endpoint_pose = None
            self._endpoint_vel = None

            return False

        if self._base_set_operational_mode:
            self._operational_mode = self._robot_operational_mode["cobot"]
        self._communication_failure = False

        return True

    def _verify_robot_state(self, now):

        if self._command_mode == self._robot_command_mode["homing"]:
            if self._enabled and not self._error and not self._communication_failure:
                if self._base_set_controller_state:
                    self._controller_state = self._robot_controller_state["motor_off"]
                return True

        if not self._ready or self._error or self._communication_failure:
            if self._base_set_controller_state:
                if self._stopped:
                    self._controller_state = self._robot_controller_state["emergency_stop"]
                elif self._error:
                    self._controller_state = self._robot_controller_state["guard_stop"]
                else:
                    self._controller_state = self._robot_controller_state["motor_off"]
            if self._error or self._command_mode != self._robot_command_mode["halt"]:
                self._command_mode = self._robot_command_mode["invalid_state"]
            return False

        if not self._enabled:
            if self._base_set_controller_state:
                self._controller_state = self._robot_controller_state["motor_off"]
            if self._command_mode != self._robot_command_mode["halt"]:
                self._command_mode = self._robot_command_mode["invalid_state"]
            return False

        if self._command_mode == self._robot_command_mode["invalid_state"] and not self._error:
             self._command_mode = self._robot_command_mode["halt"]

        if self._base_set_controller_state:
            self._controller_state = self._robot_controller_state["motor_on"]

        return True

    def _fill_robot_command(self, now):

        self._wire_position_command_sent = False
        self._wire_velocity_command_sent = False

        self._trajectory_valid = False
        self._trajectory_current_time = 0.
        self._trajectory_max_time = 0.
        self._trajectory_waypoint = 0

        if self._command_mode != self._robot_command_mode["trajectory"]:
            if self._active_trajectory is not None:
                self._active_trajectory._invalid_mode()
                self._active_trajectory = None

            if len(self._queued_trajectories) > 0:
                for t in self._queued_trajectories:
                    t._cancelled_in_queue()
                self._queued_trajectories.clear()

        if self._command_mode != self._robot_command_mode["jog"]:
            if self._jog_trajectory_generator is not None:
                self._jog_trajectory_generator = None
            if self._jog_completion_handler is not None:
                h = self._jog_completion_handler
                self._jog_completion_handler = None
                self._node.PostToThreadPool(lambda: h(None))
            
        if self._command_mode != self._robot_command_mode["velocity_command"]:
            # self._velocity_command = None
            pass
        
        if self._command_mode == self._robot_command_mode["jog"]:
            if self._jog_trajectory_generator is not None:
                jog_time = now - self._jog_start_time

                if jog_time > self._jog_trajectory_generator.t_final:
                    if self._jog_completion_handler is not None:
                        h = self._jog_completion_handler
                        self._jog_completion_handler = None
                        self._node.PostToThreadPool(lambda: h(None))
                    self._jog_trajectory_generator = None
                    return False, None, None
                
                res, jog_command = self._jog_trajectory_generator.get_command(jog_time)
                if not res:
                    return False, None, None

                joint_pos_cmd = jog_command.command_position
                return True, joint_pos_cmd, None

            else:
                if self._jog_completion_handler is not None:
                    h = self._jog_completion_handler
                    self._jog_completion_handler = None
                    self._node.PostToThreadPool(lambda: h(None))
        
                return True, None, None

        elif self._command_mode == self._robot_command_mode["position_command"]:
            
            res, pos_cmd, ts, ep = self.position_command.TryGetInValue()
            if not res:
                return True, None, None

            if self._wire_position_command_last_ep != ep:
                self._wire_position_command_last_ep = ep
                self._wire_position_command_last_seqno = 0

            if pos_cmd is None \
                or pos_cmd.seqno < self._wire_position_command_last_seqno \
                or abs(pos_cmd.state_seqno - self._state_seqno) > 10 \
                or len(pos_cmd.command) != self._joint_count \
                or len(pos_cmd.units) != 0 and len(pos_cmd.units) != self._joint_count:
                    return True, None, None
            
            pos_cmd_j = None
            if len(pos_cmd.units) == 0:
                pos_cmd_j = pos_cmd.command
            else:
                pos_cmd_j = np.zeros((self._joint_count,))
                for i in range(self._joint_count):
                    if pos_cmd.units[i] == self._joint_position_units["implicit"] \
                        or pos_cmd.units[i] == self._joint_position_units["radian"]:
                        pos_cmd_j[i] = pos_cmd.command[i]
                    elif pos_cmd.units[i] == self._joint_position_units["degree"]:
                        pos_cmd_j[i] = np.deg2rad(pos_cmd.command[i])
                    elif pos_cmd.units[i] == self._joint_position_units["ticks_rot"]:
                        pos_cmd_j[i] = pos_cmd.command[i]*(2.*np.pi)/(pow(2.,20.))
                    elif pos_cmd.units[i] == self._joint_position_units["nanoticks_rot"]:
                        pos_cmd_j[i] = pos_cmd.command[i]*(2.*np.pi)/(pow(2.,20.)*1.e9)
                    else:
                        return True, None, None

            self._wire_position_command_last_seqno = pos_cmd.seqno
            self._wire_position_command_sent = True
            return True, pos_cmd_j, None

        elif self._command_mode == self._robot_command_mode["velocity_command"]:
            
            res, vel_cmd, ts, ep = self.velocity_command.TryGetInValue()
            if not res:
                return True, None, None

            if self._wire_velocity_command_last_ep != ep:
                self._wire_velocity_command_last_ep = ep
                self._wire_velocity_command_last_seqno = 0

            if vel_cmd is None \
                or vel_cmd.seqno < self._wire_velocity_command_last_seqno \
                or abs(vel_cmd.stat_seqno - self._state_seqno) > 10 \
                or len(vel_cmd.command) != self._joint_count \
                or len(vel_cmd.units) != 0 and len(vel_cmd.units) != self._joint_count:
                    return True, None, None
            
            vel_cmd_j = None
            if len(vel_cmd.units) == 0:
                vel_cmd_j = vel_cmd.command
            else:
                vel_cmd_j = np.zeros((self._joint_count,))
                for i in range(self._joint_count):
                    if vel_cmd.units[i] == self._joint_position_units["implicit"] \
                        or vel_cmd.units[i] == self._joint_position_units["radian_second"]:
                        vel_cmd_j[i] = vel_cmd.command[i]
                    elif vel_cmd.units[i] == self._joint_position_units["degree_second"]:
                        vel_cmd_j[i] = np.deg2rad(vel_cmd.command[i])
                    elif vel_cmd.units[i] == self._joint_position_units["ticks_rot_second"]:
                        vel_cmd_j[i] = vel_cmd.command[i]*(2.*np.pi)/(pow(2.,20.))
                    elif vel_cmd.units[i] == self._joint_position_units["nanoticks_rot_second"]:
                        vel_cmd_j[i] = vel_cmd.command[i]*(2.*np.pi)/(pow(2.,20.)*1.e9)
                    else:
                        return True, None, None

            self._wire_position_command_last_seqno = vel_cmd.seqno

            if self._speed_ratio != 1.0:
                vel_cmd_j *= self._speed_ratio

            self._wire_position_command_sent = True
            return True, None, vel_cmd_j

        elif self._command_mode == self._robot_command_mode["trajectory"]:

            if self._active_trajectory is not None:
                send_traj_cmd = False

                interp_res, traj_pos, traj_vel, traj_t, traj_max_time, traj_waypoint = self._active_trajectory._get_setpoint(now, self._joint_position)

                if interp_res == TrajectoryTaskRes.ready:
                    self._trajectory_valid = True
                    send_traj_cmd = False
                elif interp_res == TrajectoryTaskRes.first_valid_setpoint or \
                     interp_res == TrajectoryTaskRes.valid_setpoint:

                     self._trajectory_valid = True
                     send_traj_cmd = True
                elif interp_res == TrajectoryTaskRes.trajectory_complete:
                    self._trajectory_valid = True
                    send_traj_comd = True
                    self._active_trajectory = None
                    if len(self._queued_trajectories) > 0:
                        self._active_trajectory = self._queued_trajectories.pop(0)
                else:
                    self._trajectory_valid = False
                    send_traj_cmd = False
                    self._active_trajectory = None
                    for w in self._queued_trajectories:
                        w._cancelled_in_queue()
                    self._queued_trajectories.clear()
                
                if self._trajectory_valid:
                    self._trajectory_current_time = traj_t
                    self._trajectory_max_time = traj_max_time
                    self._trajectory_waypoint = traj_waypoint

                if send_traj_cmd:
                    joint_pos_cmd = traj_pos
                else:
                    joint_pos_cmd = None

            else:
                joint_pos_cmd = None
            return True, joint_pos_cmd, None
        
        else:
            return True, None, None

    @abstractmethod
    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        pass
                        
    @property
    def command_mode(self):
        with self._lock:
            return self._command_mode

    @command_mode.setter
    def command_mode(self, value):
        with self._lock:
            if self._command_mode == self._robot_command_mode["invalid_state"] \
                and value == self._robot_command_mode["homing"]:

                if not self._enabled or self._communication_failure:
                    raise RR.InvalidOperationException("Cannot set homing command mode in current state")

                self._command_mode = self._robot_command_mode["homing"]
                return

            if self._command_mode == self._robot_command_mode["invalid_state"] \
                and value == self._robot_command_mode["halt"] and self._enabled and not self._error \
                and not self._communication_failure:

                self._command_mode = value
                return

            if not self._ready or self._communication_failure:
                raise RR.InvalidOperationException("Cannot set robot command mode in current state")

            if self._command_mode != self._robot_command_mode["halt"] and value != self._robot_command_mode["halt"]:
                raise RR.InvalidOperationException("Must switch to \"halt\" before selecting new mode")

            if value == self._robot_command_mode["jog"]:
                if not self._has_jog_command:
                    raise RR.InvalidOperationException("Robot does not support jog command mode")
                self._jog_trajectory_generator = None
                self._command_mode = self._robot_command_mode["jog"]
            elif value == self._robot_command_mode["halt"]:
                self._command_mode = value
            elif value == self._robot_command_mode["homing"]:
                if not self._uses_homing:
                    raise RR.InvalidOperationException("Robot does not support homing command mode")
                self._command_mode = value
            elif value == self._robot_command_mode["position_command"]:
                if not self._has_position_command:
                    raise RR.InvalidOperationException("Robot does not support position command mode")
                self._command_mode = value
            elif value == self._robot_command_mode["velocity_command"]:
                if not self._has_velocity_command:
                    raise RR.InvalidOperationException("Robot does not support velocity command mode")
                self._command_mode = value
            elif value == self._robot_command_mode["trajectory"]:
                self._command_mode = value
            else:
                raise RR.InvalidOperationException("Invalid command mode specified")
    
    def async_jog_freespace(self, joint_position, max_velocity, wait, handler):

        with self._lock:

            if self._command_mode != self._robot_command_mode["jog"]:
                raise RR.InvalidOperationException("Robot not in jog mode")

            if not self._ready:
                raise RR.InvalidOperationException("Robot not ready")

            if len(joint_position) != self._joint_count:
                raise RR.InvalidArgumentException(f"joint_position array must have {self._joint_count} elements")

            if len(max_velocity) != self._joint_count:
                raise RR.InvalidArgumentException(f"max_velocity array must have {self._joint_count} elements")

            
            if np.any(np.abs(self._joint_position - joint_position) > self._jog_joint_limit):
                raise RR.InvalidArgumentException("Position command must be within 15 degrees from current")

            if np.any(max_velocity <= 0):
                raise RR.InvalidArgumentException("max_velocity must be greater than zero")

            if self._jog_completion_handler is not None:
                h = self._jog_completion_handler
                self._jog_completion_handler = None
                self._node.PostToThreadPool(
                    lambda: h(RR.OperationAbortedException("Operation interrupted by new jog command")))
            
            now = self._stopwatch_ellapsed_s()
            if self._jog_trajectory_generator is None:
                if self._operational_mode == self._robot_operational_mode["manual_reduced_speed"]:
                    limits_a_max = np.array([j.joint_limits.reduced_acceleration for j in self._robot_info.joint_info],dtype=np.float64)
                    limits_v_max = np.array([j.joint_limits.reduced_velocity for j in self._robot_info.joint_info],dtype=np.float64)
                elif self._operational_mode == self._robot_operational_mode["manual_full_speed"] or \
                     self._operational_mode == self._robot_operational_mode["cobot"]:

                    limits_a_max = np.array([j.joint_limits.acceleration for j in self._robot_info.joint_info],dtype=np.float64)
                    limits_v_max = np.array([j.joint_limits.velocity for j in self._robot_info.joint_info],dtype=np.float64)
                else:
                    raise RR.InvalidOperationException("Invalid operation mode for jog")

                limits_x_min = np.array([j.joint_limits.lower for j in self._robot_info.joint_info],dtype=np.float64)
                limits_x_max = np.array([j.joint_limits.upper for j in self._robot_info.joint_info],dtype=np.float64)

                limits = JointTrajectoryLimits(
                    x_min = limits_x_min,
                    x_max = limits_x_max,
                    v_max = limits_v_max,
                    a_max = limits_a_max,
                    j_max = None
                    )

                for i in range(self._joint_count):
                    if np.abs(max_velocity[i]) > limits.v_max[i]:
                        raise RR.InvalidArgumentException(
                            f"max_velocity[{i}] is greater than joint limits ({limits.v_max[i]})")
                
                self._jog_trajectory_generator = TrapezoidalJointTrajectoryGenerator(self._joint_count, limits)

                new_req = JointTrajectoryPositionRequest(
                    current_position = (self._position_command if self._position_command is not None else np.copy(self._joint_position)),
                    current_velocity = (self._velocity_command if self._velocity_command is not None else np.zeros((self._joint_count,))),
                    desired_position = joint_position,
                    desired_velocity = np.zeros((self._joint_count,)),
                    max_velocity = max_velocity,
                    speed_ratio = self._speed_ratio
                )

                self._jog_trajectory_generator.update_desired_position(new_req)
                self._jog_start_time = now
            else:
                jog_trajectory_t = now - self._jog_start_time
                res, cmd = self._jog_trajectory_generator.get_command(jog_trajectory_t)
                if not res:
                    raise RR.InvalidOperationException("Cannot update jog command")

                new_req = JointTrajectoryPositionRequest(
                    current_position = cmd.command_position,
                    current_velocity = cmd.command_velocity,
                    desired_position = joint_position,
                    desired_velocity = np.zeros((self._joint_count,)),
                    max_velocity = max_velocity,
                    speed_ratio = self._speed_ratio
                )

                self._jog_trajectory_generator.update_desired_position(new_req)
                self._jog_start_time = now

            if not wait:
                self._jog_completion_source = None
            else:
                self._jog_completion_handler = handler

    
    def async_jog_joint(self, joint_velocity, timeout, wait, handler):

        with self._lock:

            if self._command_mode != self._robot_command_mode["jog"]:
                raise RR.InvalidOperationException("Robot not in jog mode")

            if not self._ready:
                raise RR.OperationAbortedException("Robot not ready")

            if len(joint_velocity) != self._joint_count:
                raise RR.InvalidArgumentException(f"joint_velocity array must have {self._joint_count} elements")

            if timeout <= 0:
                raise RR.InvalidArgumentException("Invalid jog timeout specified")

            for i in range(self._joint_count):
                if abs(joint_velocity[i] > self._robot_info.joint_info[i].joint_limits.reduced_velocity):
                    raise RR.InvalidArgumentException("Joint velocity exceeds joint limits")

            if self._jog_completion_handler is not None:
                h = self._jog_completion_handler
                self._jog_completion_handler = None
                self._node.PostToThreadPool(
                    lambda: h(RR.OperationAbortedException("Operation interrupted by new jog command")))

            now = self._stopwatch_ellapsed_s()
            if self._jog_trajectory_generator is None:
                if self._operational_mode == self._robot_operational_mode["manual_reduced_speed"]:
                    limits_a_max = np.array([j.joint_limits.reduced_acceleration for j in self._robot_info.joint_info],dtype=np.float64)
                    limits_v_max = np.array([j.joint_limits.reduced_velocity for j in self._robot_info.joint_info],dtype=np.float64)
                elif self._operational_mode == self._robot_operational_mode["manual_full_speed"] or \
                     self._operational_mode == self._robot_operational_mode["cobot"]:
                    limits_a_max = np.array([j.joint_limits.acceleration for j in self._robot_info.joint_info],dtype=np.float64)
                    limits_v_max = np.array([j.joint_limits.velocity for j in self._robot_info.joint_info],dtype=np.float64)
                else:
                    raise RR.InvalidOperationException("Invalid operation mode for jog")

                limits_x_min = np.array([j.joint_limits.lower for j in self._robot_info.joint_info],dtype=np.float64)
                limits_x_max = np.array([j.joint_limits.upper for j in self._robot_info.joint_info],dtype=np.float64)
                                
                limits = JointTrajectoryLimits(
                    x_min = limits_x_min,
                    x_max = limits_x_max,
                    v_max = limits_v_max,
                    a_max = limits_a_max,
                    j_max = None
                )

                self._jog_trajectory_generator = TrapezoidalJointTrajectoryGenerator(self._joint_count, limits)

                new_req = JointTrajectoryVelocityRequest(
                    current_position = (self._position_command if self._position_command is not None else np.copy(self._joint_position)),
                    current_velocity = (self._velocity_command if self._velocity_command is not None else np.zeros((self._joint_count,))),
                    desired_velocity = joint_velocity,
                    speed_ratio = self._speed_ratio,
                    timeout = timeout
                )

                self._jog_trajectory_generator.update_desired_velocity(new_req)
                self._jog_start_time = now
            else:
                jog_trajectory_t = now - self._jog_start_time
                res, cmd = self._jog_trajectory_generator.get_command(jog_trajectory_t)
                if not res:
                    raise RR.InvalidOperationException("Cannot update jog command")

                new_req = JointTrajectoryVelocityRequest(
                    current_position = cmd.command_position,
                    current_velocity = cmd.command_velocity,
                    desired_velocity = joint_velocity,
                    timeout = timeout,
                    speed_ratio = self._speed_ratio
                )

                self._jog_trajectory_generator.update_desired_position(new_req)
                self._jog_start_time = now

            if not wait:
                self._jog_completion_source = None
            else:
                self._jog_completion_handler = handler

    @property
    def robot_info(self):
        with self._lock:
            for i in range(len(self._robot_info.chains)):
                self._robot_info.chains[i].current_tool = self._current_tool[i]
                self._robot_info.chains[i].current_payload = self._current_payload[i]
            return self._robot_info

    def execute_trajectory(self, trajectory):
        owner_ep = RR.ServerEndpoint.GetCurrentEndpoint()

        with self._lock:
            speed_ratio = self._speed_ratio
            current_joint_pos = np.copy(self._joint_position)

        interp = JointTrajectoryInterpolator(self._robot_info)
        interp.load_trajectory(trajectory, speed_ratio)

        res, joint_pos1, _ = interp.interpolate(0)
        assert res

        if np.any(np.abs(current_joint_pos - joint_pos1) > self._trajectory_error_tol):
            raise RR.InvalidArgumentException("Starting waypoint too far from current joint positions")

        with self._lock:
            if self._command_mode != self._robot_command_mode["trajectory"]:
                raise RR.InvalidOperationException("Robot must be in trajectory mode to execut trajectory")

            traj_task = None

            if self._active_trajectory is None:
                traj_task = TrajectoryTask(self, interp, False, owner_ep)
                self._active_trajectory = traj_task
            else:
                traj_task = TrajectoryTask(self, interp, True, owner_ep)
                self._queued_trajectories.append(traj_task)

            return traj_task

    def _cancel_trajectory(self, trajectory):

        with self._lock:
            if trajectory is self._active_trajectory:
                self._active_trajectory = None
                for t in self._queued_trajectories:
                    t._cancelled_in_queue()
                self._queued_trajectories.clear()
            else:
                for i in range(len(self._queued_trajectories)):
                    if trajectory is self._queued_trajectories[i]:
                        t_index = i
                        break
                
                if t_index >= 0:
                    for i in range(len(self._queued_trajectories)-1, t_index, -1):
                        self._queued_trajectories[i]._cancelled_in_queue()
                        self._queued_trajectories.pop(i)
                    self._queued_trajectories.pop(t_index)

    def _abort_trajectory(self, trajectory):
        self._command_mode = self._robot_command_mode["halt"]

    @property
    def speed_ratio(self):
        return self._speed_ratio

    @speed_ratio.setter
    def speed_ratio(self, value):
        if value < 0.1 or value > 10:
            raise RR.InvalidArgumentException("Invalid speed_ratio")

        self._speed_ratio = value

    @property
    def operational_mode(self):        
        return self._operation_mode

    def controller_state(self):
        return self._controller_state

    def current_errors(self):
        return []

    def jog_cartesian(self, velocity, timout, wait):
        raise RR.NotImplementedException("Not implemented")

    def async_home(self, handler):
        raise RR.NotImplementedException()

    def async_getf_signal(self, handler):
        raise RR.NotImplementedException()

    def async_setf_signal(self, value, handler):
        raise RR.NotImplementedException()

    def tool_attached(self, chain, tool):
        if tool is None:
            raise RR.NullValueException("Tool cannot be null")

        if chain > 0 or not (chain < len(self._current_tool)):
            raise RR.InvalidArgumentException(f"Invalid kinematic chain {chain} for tool")

        with self._lock:
            if self._current_tool[chain] is not None:
                raise RR.InvalidArgumentException(f"Tool already attached to kinematic chain {chain}")

            self._current_tool[chain] = tool

            try:
                device_name = tool.device_info.device.name
            except:
                traceback.print_exc()
                device_name = ""

            self.tool_changed.fire(chain, device_name)
            self._config_seqno+=1

    def tool_detached(self, chain, tool_name):

        if chain > 0 or not (chain < len(self._current_tool)):
            raise RR.InvalidArgumentException(f"Invalid kinematic chain {chain} for tool")

        with self._lock:
            if self._current_tool[chain] is None:
                raise RR.InvalidArgumentException(f"Tool not attached to kinematic chain {chain}")

            if self._current_payload[chain] is None:
                raise RR.InvalidArgumentException(f"Cannot remove tool while payload attached")

            if len(tool_name) > 0:                
                try:
                    device_name = self._current_tool.device_info.device.name
                except:
                    traceback.print_exc()
                    device_name = ""
                
                if device_name != tool_name:
                    raise RR.InvalidArgumentException(f"Invalid tool name to detach from kinematic chain {chain}")
            
            self._current_tool[chain] = None

            self.tool_changed.fire(chain, "")
            self._config_seqno+=1

    def payload_attached(self, chain, payload):
        if payload is None:
            raise RR.NullValueException("Payload cannot be null")

        if chain > 0 or not (chain < len(self._current_payload)):
            raise RR.InvalidArgumentException(f"Invalid kinematic chain {chain} for payload")

        with self._lock:
            if self._current_tool[chain] is None:
                raise RR.InvalidArgumentException(f"No tool attached to kinematic chain {chain}, cannot attach payload")

            if self._current_payload[chain] is not None:
                raise RR.InvalidArgumentException(f"Payload already attached to kinematic chain {chain}")

            self._current_payload[chain] = payload

            try:
                device_name = payload.device_info.device.name
            except:
                traceback.print_exc()
                device_name = ""

            self.payload_changed.fire(chain, device_name)
            self._config_seqno+=1
    
    def payload_detached(self, chain, payload_name):
        
        if chain > 0 or not (chain < len(self._current_payload)):
            raise RR.InvalidArgumentException(f"Invalid kinematic chain {chain} for payload")

        with self._lock:
            
            if self._current_payload[chain] is None:
                raise RR.InvalidArgumentException(f"Payload not attached to kinematic chain {chain}")

            if len(payload_name) != 0:
                try:
                    device_name = self._current_payload[chain].device_info.device.name
                except:
                    traceback.print_exc()
                    device_name = ""
            
                if device_name != payload_name:
                    raise RR.InvalidArgumentException(f"Invalid payload name to detach from kinematic chain {chain}")
            
            self._current_payload[chain] = None
            self.payload_changed.fire(chain, "")
            self._config_seqno+=1

    def getf_param(self, param_name):
        raise RR.InvalidArgumentException("Invalid parameter")

    def setf_param(self, param_name, value):
        raise RR.InvalidArgumentException("Invalid parameter")

    @property
    def device_info(self):
        return self._robot_info.device_info

    @property
    def isoch_info(self):
        iso_info = self._isoch_info_type()
        iso_info.update_rate = 1.0/self._update_period
        iso_info.max_downsample = 1000

        iso_info.isoch_epoch = self._stopwatch_epoch

        return iso_info

    @property
    def isoch_downsample(self):
        with self._lock:
            return self._broadcast_downsampler.GetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint())

    @isoch_downsample.setter
    def isoch_downsample(self, value):
        with self._lock:
            self._broadcast_downsampler.SetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint(), value)


class TrajectoryTaskRes(Enum):
    unknown = 0
    ready = 1
    first_valid_setpoint = 2
    valid_setpoint = 3
    trajectory_complete = 4
    invalid_state = 5
    joint_tol_error = 6
    failed = 7


class TrajectoryTask:

    def __init__(self, parent, path, queued, owner_ep):
        self._parent = parent
        self._path = path
        self._queued = queued
        self._owner_ep = owner_ep

        self._next_called = False
        self._started = False
        self._start_time = 0
        self._aborted = False
        self._cancelled = False
        self._joint_tol_error = False
        self._finished = False
        self._next_wait_handler = []
        self._queue_wait_handler = []
        self._success_sent = False

        self._node = parent._node
        self._trajectory_status_type = \
            self._node.GetStructureType("com.robotraconteur.robotics.trajectory.TrajectoryStatus")

        self._action_consts = self._node.GetConstants("com.robotraconteur.action")
        self._action_status_code = self._action_consts["ActionStatusCode"]

        self._traj_t = 0.0
        self._traj_waypoint = 0

        self._lock = threading.Lock()

    def _call_next_wait_handler(self, err):
        with self._lock:
            for c in self._next_wait_handler:
                self._node.PostToThreadPool(lambda c=c, err=err: c(err))
            self._next_wait_handler.clear()

    def _call_queue_wait_handler(self,err):
        with self._lock:
            for c in self._queue_wait_handler:
                self._node.PostToThreadPool(lambda c=c, err=err: c(err))
            self._next_wait_handler.clear()

    def Abort(self):
        self._aborted = True
        self._parent._abort_trajectory(self)
        self._call_next_wait_handler(RR.OperationAbortedException("Trajectory execution aborted"))

    def Close(self):
        self._cancelled = True
        self._parent._cancel_trajectory(self)
        self._call_next_wait_handler(RR.OperationAbortedException("Trajectory execution cancelled"))

    def AsyncNext(self,handler):
        if self._success_sent:
            raise RR.StopIterationException("")

        with self._lock:            
            first_call = not self._next_called
            self._next_called = True

            if first_call and self._queued:
                # Report back that we are queued immediately

                ret = self._trajectory_status_type()
                ret.action_status = self._action_status_code["queued"]
                ret.trajectory_time = 0
                ret.current_waypoint = 0
                ret.seqno = self._parent._state_seqno
                handler(ret, None)
                return

            complete_called = [False]

            def complete(err):
                with self._lock:
                    if complete_called[0]:
                        return
                    complete_called[0] = True
                if err:
                    handler(None, err)
                
                if not self._started:
                    # Still queued...

                    ret = self._trajectory_status_type()
                    ret.action_status = self._action_status_code["queued"]
                    ret.trajectory_time = 0
                    ret.current_waypoint = 0
                    ret.seqno = self._parent._state_seqno
                    handler(ret, None)
                    return

                if self._finished:
                    self._success_sent = True
                    ret = self._trajectory_status_type()
                    ret.action_status = self._action_status_code["complete"]
                    ret.trajectory_time = self._traj_t
                    ret.current_waypoint = int(self._traj_waypoint)
                    ret.seqno = self._parent._state_seqno
                    handler(ret, None)
                    return

                else:
                    ret = self._trajectory_status_type()
                    ret.action_status = self._action_status_code["running"]
                    ret.trajectory_time = self._traj_t
                    ret.current_waypoint = int(self._traj_waypoint)
                    ret.seqno = self._parent._state_seqno
                    handler(ret,None)
                    return

            if self._queued:
                self._next_wait_handler.append(complete)
                self._queue_wait_handler.append(complete)
            else:
                self._next_wait_handler.append(complete)

            timer = self._node.CreateTimer(5, lambda _: complete(None), True)
            timer.Start()

    def _cancelled_in_queue(self):
        self._cancelled = True
        self._call_next_wait_handler(RR.OperationAbortedException("Trajectory cancelled by controller before start"))

    def _invalid_mode(self):
        self._aborted = True
        self._call_next_wait_handler(RR.OperationAbortedException("Invalid mode for trajectory execution"))

    
    def _get_setpoint(self, now, current_joint_pos):
        if self._cancelled or self._aborted:
            return TrajectoryTaskRes.failed, None, None, 0.0, 0.0, 0

        first_call = False

        t = 0.0

        if self._next_called:
            if not self._started:
                self._start_time = now
                self._started = True
                first_call = True

            t = now - self._start_time

        res, joint_pos1, current_waypoint1 = self._path.interpolate(t)
        if not res:
            self._call_next_wait_handler(Exception("Trajectory execution failed"))
            return TrajectoryTaskRes.failed, None, None, 0.0, 0.0, 0

        if np.any(np.abs(current_joint_pos - joint_pos1) > self._parent._trajectory_error_tol):
            self._call_next_wait_handler(RR.OperationFailedException("Trajectory tolerance failure"))
            return TrajectoryTaskRes.ready, None, None, 0.0, 0.0, 0

        if not self._next_called:
            return TrajectoryTaskRes.ready, None, None, 0.0, self._path.max_time, 0
        
        if t > self._path.max_time:
            self._traj_t = t
            self._traj_waypoint = current_waypoint1
            self._finished = True
            self._call_next_wait_handler(None)
            return TrajectoryTaskRes.trajectory_complete, joint_pos1, None, t, self._path.max_time, current_waypoint1

        if first_call:
            if self._queued:
                self._queued = False
                self._call_queue_wait_handler(None)
            return TrajectoryTaskRes.first_valid_setpoint, joint_pos1, None, t, self._path.max_time, current_waypoint1
        else:
            return TrajectoryTaskRes.valid_setpoint, joint_pos1, None, t, self._path.max_time, current_waypoint1

    #TODO: Add connection test?

    