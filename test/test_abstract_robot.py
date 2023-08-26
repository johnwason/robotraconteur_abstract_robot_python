import os
import numpy as np
from robotraconteur_abstract_robot import AbstractRobot
import RobotRaconteur as RR
import RobotRaconteurCompanion as RRC
import numpy.testing as nptest
import time
from RobotRaconteurCompanion.Util.UuidUtil import UuidUtil
import general_robotics_toolbox as rox
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil


class TestRobot(AbstractRobot):

    def __init__(self, robot_info, default_joint_count, node):
        super().__init__(robot_info, default_joint_count, node)
    
    def _send_disable(self, handler):
        pass

    def _send_enable(self, handler):
        pass

    def _send_reset_errors(self, handler):
        pass

    def _send_robot_command(self, joint_pos_cmd, joint_vel_cmd):
        pass

def _get_absolute_path(fname):
    dirname = os.path.dirname(os.path.realpath(__file__))
    return dirname + "/" + fname

def _new_test_node(std_robdef=False,server=False,nodename=None):
    node = RR.RobotRaconteurNode()
    if nodename is not None:
        node.SetNodeName(nodename)
    node.Init()
    if std_robdef:
        RRC.RegisterStdRobDefServiceTypes(node)
    t = RR.IntraTransport(node)
    node.RegisterTransport(t)
    if server:
        t.StartServer()
    node.SetLogLevelFromString("WARNING")
    return node

def _init_test_robot_obj(node):    
    info_parser = RRC.InfoParser(node=node)
    with open(_get_absolute_path("abb_1200_5_90_robot_default_config.yml")) as f:
        info_str = f.read()
    robot_info = info_parser.ParseInfoString(info_str, "com.robotraconteur.robotics.robot.RobotInfo")

    obj = TestRobot(robot_info, 0, node)
    return obj

class _test_robot_container:
    def __init__(self):
        self.node = _new_test_node(True,True,"robot")
        self.client_node = _new_test_node()
        self.robot_obj = _init_test_robot_obj(self.node)
        self.node.RegisterService("robot", "com.robotraconteur.robotics.robot.Robot", self.robot_obj)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.robot_obj._close()
        self.node.Shutdown()
        self.client_node.Shutdown()

    def connect_client(self):
        return self.node.ConnectService("rr+intra:///?nodename=robot&service=robot")

def _test_serialize(obj, rr_type):    
    node = _new_test_node(std_robdef=True)
    try:
        from RobotRaconteur.RobotRaconteurPythonUtil import PackMessageElement, UnpackMessageElement
        mm = PackMessageElement(obj, rr_type, node=node)
        mm.UpdateData()
        return UnpackMessageElement(mm, node=node)
    finally:
        node.Shutdown()

def test_init_abstract_robot():
    node = _new_test_node(std_robdef=True)
    
    try:
        obj = _init_test_robot_obj(node)
        time.sleep(0.05)
        obj._close()
    finally:
        node.Shutdown()

def test_robot_container():
    with _test_robot_container() as c:
        assert c.node is not None
        assert c.robot_obj is not None

    assert not c.robot_obj._keep_going 
    # assert c.node.IsShutdown()

def test_robot_container_client():
    with _test_robot_container() as c:
        client = c.connect_client()
        print(client.device_info)
        print(client.robot_info)

def test_robot_init():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        assert c._node is c1.node
        assert c._joint_names == [f"joint_{i+1}" for i in range(6)]
        assert c._joint_count == 6

        assert c._robot_caps == 0x1 | 0x2 | 0x4
        
        assert set(c._robot_consts.keys()) == {'RobotCapabilities', 'RobotControllerState',
         'RobotCommandMode', 'RobotOperationalMode', 'RobotStateFlags', 'RobotTypeCode'}

        assert set(c._robot_capabilities.keys()) == {'jog_command', 'trajectory_queueing', 'trajectory_command', 
            'software_reset_errors', 'software_enable', 'raster_trajectory', 'position_command', 'homing_command', 
            'interpolated_trajectory', 'velocity_command', 'speed_ratio', 'unknown'}

        assert set(c._robot_command_mode.keys()) == {'trajectory', 'homing', 'position_command', 'jog',
            'invalid_state', 'velocity_command', 'halt'}

        assert set(c._robot_operational_mode.keys()) == {'manual_full_speed', 'auto', 'undefined', 
            'manual_reduced_speed', 'cobot'}

        assert set(c._robot_controller_state.keys()) == {'emergency_stop', 'motor_off', 'motor_on', 
            'undefined', 'init', 'emergency_stop_reset', 'guard_stop'}

        assert set(c._robot_state_flags.keys()).issuperset({'estop_button2', 'homed', 'estop_released', 
            'estop_internal', 'fatal_error', 'unknown', 'trajectory_running', 'enabling_switch', 'estop_guard4', 
            'estop', 'estop_software', 'estop_fault', 'estop_other', 'ready'})

        assert set(c._joint_consts.keys()) == {'JointPositionUnits', 'JointJerkUnits', 'JointType', 
            'JointEffortUnits', 'JointVelocityUnits', 'JointAccelerationUnits'}

        assert set(c._joint_position_units.keys()) == {'nanoticks_rot', 'meter', 'implicit', 'degree', 
            'ticks_lin', 'ticks_rot', 'radian', 'nanoticks_lin'}

        assert set(c._joint_effort_units.keys()) == {'tesla', 'weber', 'volt', 'implicit', 'ampere', 'meter_second2', 
            'coulomb', 'degree_second2', 'radian_second2', 'newton_meter', 'pascal', 'newton'}

        assert not c._uses_homing 
        assert c._has_position_command
        assert not c._has_velocity_command
        assert c._has_jog_command

        assert len(c._rox_robots) == 1
        nptest.assert_allclose(c._rox_robots[0].H,
            np.array([[0., 0., 0., 1., 0., 1.],[0., 1., 1., 0., 1., 0.],[1., 0., 0., 0., 0., 0.]]))

        assert c._current_tool == [None]
        assert c._current_payload == [None]

        assert c._robot_info.joint_info[3].joint_limits.reduced_velocity == \
            c._robot_info.joint_info[3].joint_limits.velocity
        assert c._robot_info.joint_info[3].joint_limits.reduced_acceleration == \
            c._robot_info.joint_info[3].joint_limits.acceleration

        assert not c._keep_going

def test_fill_state_flags():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        c._communication_failure = True
        assert c._fill_state_flags(0) == 0x200000
        c._communication_failure = False

        c._error = True
        assert c._fill_state_flags(0) == 0x1
        c._error = False

        c._stopped = True
        assert c._fill_state_flags(0) == 0x4
        c._estop_source = 1
        assert c._fill_state_flags(0) == 0x8 | 0x4
        c._estop_source = 2
        assert c._fill_state_flags(0) == 0x4000 | 0x4
        c._estop_source = 3
        assert c._fill_state_flags(0) == 0x1000 | 0x4
        c._estop_source = 4
        assert c._fill_state_flags(0) == 0x2000 | 0x4
        c._estop_source = 0
        c._stopped = False

        c._wire_position_command_sent = True
        assert c._fill_state_flags(0) == 0x1000000
        c._wire_position_command_sent = False

        c._wire_velocity_command_sent = True
        assert c._fill_state_flags(0) == 0x2000000
        c._wire_velocity_command_sent = False

        c._trajectory_valid = True
        assert c._fill_state_flags(0) == 0x4000000
        c._trajectory_valid = False
        
def test_calc_endpoint_pose():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        assert c._calc_endpoint_poses().shape == (0,)

        geom_util = GeometryUtil(c1.node)

        pose1 = geom_util.xyz_rpy_to_pose([0.1,0.2,0.3],[0.0233,0.04,0.06])

        c._endpoint_pose = [pose1]

        assert c._calc_endpoint_poses() == pose1

        tool1 = c1.node.NewStructure("com.robotraconteur.robotics.tool.ToolInfo")
        tool1.tcp = geom_util.xyz_rpy_to_transform([0,0,0.01],[0.009,0.008,0.007])
        c._current_tool = [tool1]

        nptest.assert_allclose(c1.node.NamedArrayToArray(c._calc_endpoint_poses()).flatten(),
            [0.99903032, 0.01547613, 0.02444504, 0.03318644, 0.10041304, 0.19979141, 0.30998929], atol=1e-4)

        
def test_calc_endpoint_vel():
    
    with _test_robot_container() as c1:
        c = c1.robot_obj

        assert c._calc_endpoint_vels().shape == (0,)

        geom_util = GeometryUtil(c1.node)

        vel1 = geom_util.array_to_spatial_velocity([0.01,0.02,0.03,0.04,0.05,0.06])

        c._endpoint_vel=[vel1]

        assert c._calc_endpoint_vels() == vel1

        tool1 = c1.node.NewStructure("com.robotraconteur.robotics.tool.ToolInfo")
        tool1.tcp = geom_util.xyz_rpy_to_transform([0,0,0.01],[0.009,0.008,0.007])
        c._current_tool = [tool1]

        pose1 = geom_util.xyz_rpy_to_pose([0.1,0.2,0.3],[0.0233,0.04,0.06])

        c._endpoint_pose = [pose1]

        res1 = c1.node.NamedArrayToArray(c._calc_endpoint_vels()).flatten()
        nptest.assert_allclose(res1, [0.01      , 0.02      , 0.03      , 0.04020604, 0.0499125 ,
            0.05998965], atol=1e-4)

def test_fill_states():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        rob_state, a_rob_state, sensor_data = c._fill_states(time.perf_counter())

        c._state_seqno=59763
        c._command_mode=3
        c._operational_mode=1
        c._controller_state=3

        c._enabled=True
        c._ready=True
        c._wire_position_command_sent=True
        c._communication_failure=False

        c._joint_position=np.array([0.5379, 0.4294, 0.4161, 0.7159, 0.4958, 0.4698])
        c._joint_velocity=np.array([0.1378, 0.7312, 0.044 , 0.5284, 0.1485, 0.494 ])
        c._joint_effort=np.array([0.2419, 0.9546, 0.6668, 0.1725, 0.5579, 0.9471])
        c._position_command=np.array([0.2441, 0.9979, 0.3714, 0.723 , 0.8448, 0.7268])
        c._velocity_command=np.array([0.1381, 0.9104, 0.0705, 0.091 , 0.643 , 0.4823])

        geom_util = GeometryUtil(c1.node)

        pose1 = geom_util.xyz_rpy_to_pose([0.1,0.2,0.3],[0.0233,0.04,0.06])
        c._endpoint_pose = [pose1]
        vel1 = geom_util.array_to_spatial_velocity([0.01,0.02,0.03,0.04,0.05,0.06])
        c._endpoint_vel=[vel1]

        c._trajectory_current_time = 0.0247
        c._trajectory_max_time = 0.628
        c._trajectory_waypoint = 352

        

        rob_state, a_rob_state, sensor_data = c._fill_states(time.perf_counter())

        assert rob_state.ts is not None
        assert rob_state.seqno == 59763
        assert rob_state.command_mode == 3
        assert rob_state.operational_mode == 1
        assert rob_state.controller_state == 3
        assert rob_state.robot_state_flags == 0x20000 | 0x40000 | 0x1000000
        nptest.assert_allclose(rob_state.joint_position, [0.5379, 0.4294, 0.4161, 0.7159, 0.4958, 0.4698])
        nptest.assert_allclose(rob_state.joint_velocity,[0.1378, 0.7312, 0.044 , 0.5284, 0.1485, 0.494 ])
        nptest.assert_allclose(rob_state.joint_effort,[0.2419, 0.9546, 0.6668, 0.1725, 0.5579, 0.9471])
        nptest.assert_allclose(rob_state.joint_position_command,[0.2441, 0.9979, 0.3714, 0.723 , 0.8448, 0.7268])
        nptest.assert_allclose(rob_state.joint_velocity_command,[0.1381, 0.9104, 0.0705, 0.091 , 0.643 , 0.4823])
        assert rob_state.kin_chain_tcp == pose1
        assert rob_state.kin_chain_tcp_vel == vel1
        assert not rob_state.trajectory_running

        assert a_rob_state.ts is not None
        assert a_rob_state.seqno == 59763
        assert a_rob_state.command_mode == 3
        assert a_rob_state.operational_mode == 1
        assert a_rob_state.controller_state == 3
        assert a_rob_state.robot_state_flags == 0x20000 | 0x40000 | 0x1000000
        nptest.assert_allclose(a_rob_state.joint_position, [0.5379, 0.4294, 0.4161, 0.7159, 0.4958, 0.4698])
        nptest.assert_allclose(a_rob_state.joint_velocity,[0.1378, 0.7312, 0.044 , 0.5284, 0.1485, 0.494 ])
        nptest.assert_allclose(a_rob_state.joint_effort,[0.2419, 0.9546, 0.6668, 0.1725, 0.5579, 0.9471])
        nptest.assert_allclose(a_rob_state.joint_position_command,[0.2441, 0.9979, 0.3714, 0.723 , 0.8448, 0.7268])
        nptest.assert_allclose(a_rob_state.joint_velocity_command,[0.1381, 0.9104, 0.0705, 0.091 , 0.643 , 0.4823])
        nptest.assert_array_equal(a_rob_state.joint_position_units, [2]*6)
        nptest.assert_array_equal(a_rob_state.joint_effort_units, [65]*6)
        assert a_rob_state.kin_chain_tcp == pose1
        assert a_rob_state.kin_chain_tcp_vel == vel1
        assert not a_rob_state.trajectory_running
        assert a_rob_state.trajectory_time == 0.0247
        assert a_rob_state.trajectory_max_time == 0.628
        assert a_rob_state.trajectory_current_waypoint == 352
        assert a_rob_state.config_seqno == 1

        assert sensor_data.robot_state.ts is not None
        assert sensor_data.robot_state.seqno == 59763
        assert sensor_data.robot_state.command_mode == 3
        assert sensor_data.robot_state.operational_mode == 1
        assert sensor_data.robot_state.controller_state == 3
        assert sensor_data.robot_state.robot_state_flags == 0x20000 | 0x40000 | 0x1000000
        nptest.assert_allclose(sensor_data.robot_state.joint_position, [0.5379, 0.4294, 0.4161, 0.7159, 0.4958, 0.4698])
        nptest.assert_allclose(sensor_data.robot_state.joint_velocity,[0.1378, 0.7312, 0.044 , 0.5284, 0.1485, 0.494 ])
        nptest.assert_allclose(sensor_data.robot_state.joint_effort,[0.2419, 0.9546, 0.6668, 0.1725, 0.5579, 0.9471])
        nptest.assert_allclose(sensor_data.robot_state.joint_position_command,[0.2441, 0.9979, 0.3714, 0.723 , 0.8448, 0.7268])
        nptest.assert_allclose(sensor_data.robot_state.joint_velocity_command,[0.1381, 0.9104, 0.0705, 0.091 , 0.643 , 0.4823])
        nptest.assert_array_equal(sensor_data.robot_state.joint_position_units, [2]*6)
        nptest.assert_array_equal(sensor_data.robot_state.joint_effort_units, [65]*6)
        assert sensor_data.robot_state.kin_chain_tcp == pose1
        assert sensor_data.robot_state.kin_chain_tcp_vel == vel1
        assert not sensor_data.robot_state.trajectory_running
        assert sensor_data.robot_state.trajectory_time == 0.0247
        assert sensor_data.robot_state.trajectory_max_time == 0.628
        assert sensor_data.robot_state.trajectory_current_waypoint == 352
        assert sensor_data.robot_state.config_seqno == 1

        assert sensor_data.data_header.seqno == c._state_seqno

def test_verify_communication():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        now = time.perf_counter()

        assert not c._verify_communication(now)
        assert c._communication_failure
        assert c._command_mode == -1
        assert c._operational_mode == 0
        assert c._controller_state == 0
        assert np.all(c._joint_position == np.zeros((0,)))
        assert np.all(c._joint_velocity == np.zeros((0,)))
        assert np.all(c._joint_effort == np.zeros((0,)))
        assert c._endpoint_pose is None
        assert c._endpoint_vel is None

        last_val = now - 0.005
        c._last_joint_state = last_val
        c._last_robot_state = last_val
        c._last_endpoint_state = last_val
        assert c._verify_communication(now)
        assert not c._communication_failure
        assert c._operational_mode == 4

def test_verify_robot_state():
    with _test_robot_container() as c1:
        c = c1.robot_obj

        now = time.perf_counter()

        c._command_mode = 5
        c._enabled = True
        c._communication_failure = False
        assert c._verify_robot_state(now)
        assert c._controller_state ==3

        c._communication_failure = False
        c._error = True
        assert not c._verify_robot_state(now)
        assert c._controller_state == 4
        assert c._command_mode == -1
        c._stopped = True
        assert not c._verify_robot_state(now)
        assert c._controller_state == 5
        c._stopped = False
        c._error = False
        c._communication_failure = True
        c._error = False
        assert not c._verify_robot_state(now)
        assert c._controller_state == 3
        assert c._command_mode == -1
        c._communication_failure = False
        c._enabled = False
        assert not c._verify_robot_state(now)
        assert c._controller_state == 3
        assert c._command_mode == -1
        c._enabled = True
        c._ready = True
        assert c._verify_robot_state(now)
        assert c._controller_state == 2
        assert c._command_mode == 0

#TODO: more tests on AbstractRobot

def test_robot_info():
    with _test_robot_container() as c1:
        c = c1.robot_obj
        robot_info = c.robot_info

        _test_serialize(robot_info.device_info, "com.robotraconteur.device.DeviceInfo")
        _test_serialize(robot_info, "com.robotraconteur.robotics.robot.RobotInfo")

