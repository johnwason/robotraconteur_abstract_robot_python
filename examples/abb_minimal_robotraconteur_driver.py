# Minimal Robot Raconteur driver for ABB robots using Externally Guided Motion
# See abb_robotraconteur_driver_hmp for a more sophisticated driver example

# Before running, install driver dependencies:
#
#    pip install robotraconteur-abstract-robot abb-robot-client
#
# Optionally do an editable install of robotraconteur-abstract-robot before above command. Run in the root directory of 
# the robotraconteur_abstract_robot_python repo:
#
#    pip install -e .
#

# Robot must have EGM configured and be running the RAPID code in abb_minimal_robotraconteur_driver_rapid.mod
#
# Run driver with:
#
#    python abb_minimal_robotraconteur_driver --robot-info=abb_1200_5_90_robot_default_config.yml

from contextlib import suppress
import sys
import time
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import numpy as np
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.RobDef import register_service_types_from_resources
from abb_robot_client.egm import EGM

import argparse

from robotraconteur_abstract_robot import AbstractRobot

class ABBRobotMinimalImpl(AbstractRobot):
    def __init__(self, robot_info):
        # Call AbstractRobot __init__
        super().__init__(robot_info, 6)
        # This driver does not home the robot
        self._uses_homing = False
        # Streaming position command is available
        self._has_position_command = True
        # ABB robots do not support external streaming velocity command
        self._has_velocity_command = False
        # Use 250 Hz update loop (4 ms timestep)
        self._update_period = 4e-3
        # EGM does not provide controller state or operational mode. Other drivers that use Robot Web Services (RWS)
        # do provide this information
        self._base_set_controller_state = True
        self._base_set_operational_mode = True
        # Override device capabilities from RobotInfo
        self.robot_info.robot_capabilities &= self._robot_capabilities["jog_command"] \
             & self._robot_capabilities["position_command"] & self._robot_capabilities["trajectory_command"] \
        # Set trajectory_error_tol to a large number. Due to high latency in ABB communication this has not been useful
        self._trajectory_error_tol = 1000

        self._egm = None

    def _start_robot(self):
        # Create EGM client
        self._egm = EGM()
        super()._start_robot()
        time.sleep(0.5)

    def _run_timestep(self, now):
        res = True
        robot_state = None
        while res:
            res, robot_state1 = self._egm.receive_from_robot()
            if res:
                robot_state = robot_state1
        
        if robot_state is not None:
            egm_last_recv = self._stopwatch_ellapsed_s()
            self._last_joint_state = egm_last_recv
            self._last_endpoint_state = egm_last_recv
            self._last_robot_state = egm_last_recv
            self._enabled = robot_state.motors_on
            self._ready = robot_state.rapid_running
        
            self._joint_position = np.deg2rad(robot_state.joint_angles)            
            self._endpoint_pose = self._node.ArrayToNamedArray(\
                np.concatenate((robot_state.cartesian[1],robot_state.cartesian[0]*1e-3)), self._pose_dtype)            
        else:
            if self._communication_failure:
                self._joint_position = np.zeros((0,))        
                self._endpoint_pose = np.zeros((0,),dtype=self._pose_dtype)

        super()._run_timestep(now)

        # For this driver, we save the _position_command and send here
        if self._egm.egm_addr is not None:
            if self._position_command is None:
                self._egm.send_to_robot(None)
            else:
                self._egm.send_to_robot(np.rad2deg(self._position_command))

    def _send_robot_command(self, now, joint_pos_cmd, joint_vel_cmd):
        # Save position command and send as part of _run_timestep()
        if joint_pos_cmd is not None:
            self._position_command = joint_pos_cmd
        else:
            self._position_command = None

    def _send_disable(self, handler):
        raise RR.NotImplementedException("Unsupported function")

    def _send_enable(self, handler):
        raise RR.NotImplementedException("Unsupported function")

    def _send_reset_errors(self, handler):
        raise RR.NotImplementedException("Unsupported function")

def main():

    # Read and parse command line arguments for service. Arguments starting with ``--robotraconteur-`` are passed
    # to the node setup
    parser = argparse.ArgumentParser(description="ABB minimal example robot driver service for Robot Raconteur")
    parser.add_argument("--robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Robot info file (required)")
    parser.add_argument("--robot-name", type=str,default=None,help="Optional device name override")
    args, _ = parser.parse_known_args()

    # Register standard Robot Raconteur service types with the node
    RRC.RegisterStdRobDefServiceTypes(RRN)
    
    # Read the RobotInfo yaml file to string
    with args.robot_info_file:
        robot_info_text = args.robot_info_file.read()

    # Parse the RobotInfo file
    info_loader = InfoFileLoader(RRN)
    robot_info, robot_ident_fd = info_loader.LoadInfoFileFromString(robot_info_text, "com.robotraconteur.robotics.robot.RobotInfo", "device")

    # Create the node attributes from RobotInfo. These attributes are made available during Robot Raconteur 
    # service discovery to help clients identify services
    attributes_util = AttributesUtil(RRN)
    robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(robot_info.device_info)

    # Create the robot driver object. This object extends AbstractRobot
    robot = ABBRobotMinimalImpl(robot_info)
    try:
        
        # Call _start_robot() to start the robot loop
        robot._start_robot()
        time.sleep(0.5)

        # Use ServerNodeSetup to initialize the server node
        with RR.ServerNodeSetup("experimental.abb_robot.minimal_robot",59926,argv=sys.argv):
            
            # Register the service and add attributes from RobotInfo file
            service_ctx = RRN.RegisterService("robot","com.robotraconteur.robotics.robot.Robot",robot)
            service_ctx.SetServiceAttributes(robot_attributes)

            print("Press enter to exit...")
            input()
            robot._close()

    except:
        # Close robot if there is an error
        with suppress(Exception):
            robot._close()
        raise

if __name__ == "__main__":
    main()