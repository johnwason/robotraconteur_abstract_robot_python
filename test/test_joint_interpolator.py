import numpy as np
from robotraconteur_abstract_robot.joint_trajectory_interpolator import JointTrajectoryInterpolator
import RobotRaconteur as RR
import RobotRaconteurCompanion as RRC
import os
import numpy.testing as nptest

def _get_absolute_path(fname):
    dirname = os.path.dirname(os.path.realpath(__file__))
    return dirname + "/" + fname

def test_joint_trajectory_interpolator():
    
    joint_pos = np.array([[0.53960206, 0.01361414, 0.70883563, 0.82985313, 0.82061402, 0.67983269],
       [0.08824231, 0.00611721, 0.66442393, 0.91101476, 0.95975989, 0.66002348],
       [0.43313536, 0.03263386, 0.08653679, 0.54870427, 0.87862665, 0.61059985],
       [0.982817  , 0.65710011, 0.37762462, 0.24798666, 0.9835447, 0.61930514],
       [0.35665487, 0.7424226 , 0.02763994, 0.33406137, 0.88513656, 0.51701165],
       [0.00101731, 0.28577329, 0.80532673, 0.12878102, 0.21458496, 0.24469729],
       [0.03123814, 0.72594553, 0.61098597, 0.15131456, 0.0441739, 0.91766966],
       [0.21273568, 0.54601886, 0.60489315, 0.99939826, 0.06441626, 0.17950606],
       [0.40534227, 0.90194967, 0.09997114, 0.5183201 , 0.28885956, 0.09084113],
       [0.53010707, 0.25605409, 0.72179921, 0.51540499, 0.34549319, 0.74694832]])*0.01

    t = np.arange(0,0.1,0.01)
    
    try:
        node = RR.RobotRaconteurNode()
        node.Init()
        RRC.RegisterStdRobDefServiceTypes(node)
        info_parser = RRC.InfoParser(node=node)
        with open(_get_absolute_path("abb_1200_5_90_robot_default_config.yml")) as f:
            info_str = f.read()
        robot_info = info_parser.ParseInfoString(info_str, "com.robotraconteur.robotics.robot.RobotInfo")

        traj_waypoint_type = node.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint")
        traj_type = node.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory")

        traj_interp = JointTrajectoryInterpolator(robot_info)
        assert traj_interp._joint_names == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        nptest.assert_allclose(traj_interp._joint_min, [-2.967, -1.745, -3.491, -4.712, -2.269, -6.283])
        nptest.assert_allclose(traj_interp._joint_max, [2.967, 2.269, 1.222, 4.712, 2.269, 6.283])
        nptest.assert_allclose(traj_interp._joint_vel_max, [ 5.027,  4.189,  5.236,  6.981,  7.069, 10.472])


        j_w = []
        for i in range(len(t)):
            w = traj_waypoint_type()
            w.joint_position = joint_pos[i,:]
            w.time_from_start = t[i]
            j_w.append(w)

        traj = traj_type()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        traj.waypoints = j_w

        speed_ratio = 0.9

        traj_interp.load_trajectory(traj, speed_ratio)

        for i in range(len(t)):
            res, j, current_waypoint = traj_interp.interpolate(t[i] / speed_ratio)
            assert res
            nptest.assert_allclose(j, joint_pos[i,:])
            assert current_waypoint == i

        res, j, current_waypoint = traj_interp.interpolate(-1)
        assert res
        nptest.assert_allclose(j, joint_pos[0,:])
        assert current_waypoint == 0

        res, j, current_waypoint = traj_interp.interpolate(100)
        assert res
        nptest.assert_allclose(j, joint_pos[-1,:])
        assert current_waypoint == len(traj.waypoints) -1
    
    finally:
        node.Shutdown()