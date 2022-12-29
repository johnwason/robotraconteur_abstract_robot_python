**class robotraconteur_abstract_robot.AbstractRobot(robot_info, default_joint_count, node=None)**

   Abstact base class for standard Robot Raconteur robot device drivers. Subclasses implement specific functionality
   for each robot controller type. Typically, these drivers communicate with the vendor controller. The vender
   controller may provide the communication method natively, or the vendor controller may need to execute
   special programs provided by the driver.

   The driver uses a ``RobotInfo`` structure to initialize information about kinematics etc. The __init__
   function should also be overridden to initialize various instance variables. The ``robot_info`` parameter
   is typically loaded from a YAML file.

   AbstractRobot uses a real-time loop that periodically calls ``_run_timestep()``, with the period set by 
   ``_update_period``. ``_run_timestep()`` does the following, some of which the subclass must implement:

   1. Read feedback from driver (must be implemented by subclass).
          ..

             Update ``_joint_position``, ``_joint_velocity`` (optional), ``_joint_effort`` (optional), ``_endpoint_pose``,

          ``_endpoint_vel`` (optional), ``_ready``, ``_enabled``, ``_stopped``, ``_error``, ``_estop_source``,
          ``_last_robot_state``, ``_last_joint_state``, and ``_last_endpoint_state``. These updates may happen
          outside the loop, when the data is received from the robot. Hold ``_lock`` when updating data if not
          inside the loop.

   2. Verify communication by calling ``_verify_communication()``. If ``_last_robot_state``, ``_last_joint_state``,
          or ``_last_endpoint_state`` exceed ``_communication_timeout`` relative to stopwatch time, set communication
          failure.

   3. Verify the current robot state by calling >>``<<_verify_robot_state()

   4. Fill a joint position or joint velocity command by calling ``_fill_robot_command()``. This will check the
          current operational mode and commands from the client to generate the next command.

   5. Fill the robot state structures to return to clients. Calls ``_fill_states()``, ``_fill_state_flags()``,
          ``_calc_endpoint_poses()``, and ``_calc_endpoint_vels()``

   6. If a valid command is available, send to the robot using ``_send_robot_command()``. Subclass must implement
          this function.

   At a minimum, a driver subclass must fill feedback data from the robot as shown in step 1 above, and must
   implement ``_send_robot_command()``, ``_send_disable()``, ``_send_enable()``, and ``_send_reset_errors()``.
   See the example minimal ABB robot driver. Also see abb_robotraconteur_driver_hmp for a more sophisticated driver.

   :Variables:       
      * **_robot_info** – The ``RobotInfo`` structure, initialized from __init__ parameter

      * **_joint_names** – The names of the robot joints. Initialized from ``robot_info`` or ``default_joint_count``

      * **_joint_count** – The number of robot joints. Initialized from ``robot_info`` or ``default_joint_count``

      * **_robot_uuid** – The UUID of the robot. Initialized from the ``robot_info`` structure

      * **_robot_caps** – The capability flags of the robot taken from ``RobotCapabilities`` enum. By default initialized
        from >>``<<robot_info`, but it is recommended the driver override this value in __init__

      * **_robot_util** – Companion ``RobotUtil`` utility class instance

      * **_datetime_util** – Companion ``DateTimeUtil`` utility class instance

      * **_geometry_util** – Companion ``GeometryUtil`` utility class instance

      * **_sensor_data_util** – Companion ``SensorDataUtil`` utility class instance

      * **_pose_dtype** – ``com.robotraconteur.geometry.Pose`` numpy dtype

      * **_spatial_velocity_dtype** – ``com.robotraconteur.geometry.SpatialVelocity`` numpy dtype

      * **_robot_state_type** – ``RobotState`` structure type

      * **_advanced_robot_state_type** – ``AdvancedRobotState`` structure type

      * **_robot_state_sensor_data_type** – ``RobotStateSensorData`` structure type

      * **_robot_joint_command_type** – ``RobotJointCommand`` structure type

      * **_isoch_info_type** – ``IsochInfo`` structure type

      * **_robot_consts** – Constants from ``com.robotraconteur.robotics.robot``

      * **_robot_capabilities** – ``RobotCapabilities`` enum

      * **_robot_command_mode** – ``RobotCommandMode`` enum

      * **_robot_operational_mode** – ``RobotOperationalMode`` enum

      * **_robot_controller_state** – ``RobotControllerState`` enum

      * **_robot_state_flags** – ``RobotStateFlags`` enum

      * **_joint_consts** – Constants from ``com.robotraconteur.robotics.joints``

      * **_joint_position_units** – ``JointPositionUnits`` enum

      * **_joint_effort_units** – ``JointEffortUnits`` enum

      * **_uses_homing** – Robot uses homing command. Initialized from capabilities flags in ``robot_info``. 
        Recommended to override in __init__

      * **_has_position_command** – Robot has streaming position command. Initialized from capabilities flags in 
        ``robot_info``. Recommended to override in __init__

      * **_has_velocity_command** – Robot has streaming velocity command. Initialized from capabilities flags in 
        ``robot_info``. Recommended to override in __init__

      * **_has_jog_command** – Robot has jog command. Initialized from capabilities flags in 
        ``robot_info``. Recommended to override in __init__

      * **_current_tool** – Currently attached robot tool. Array, one entry per chain. Initialized from ``robot_info``,
        updated using ``tool_attached()`` and ``tool_detached()``

      * **_current_payload** – Currently attached payload. Array, one entry per chain. Initialized from ``robot_info``,
        updated using ``payload_attached()`` and ``payload_detached()``

      * **_current_payload_pose** – Pose of currently attached payload relative to tool TCP. Array, one entry per chain. 
        Initialized from ``robot_info``, updated using ``payload_attached()`` 
        and ``payload_detached()``

      * **_keep_going** – Boolean flag to stop loop

      * **_update_period** – The update period of the loop (aka timestep). Should be set in __init__

      * **_speed_ratio** – The current speed ratio. Set using ``speed_ratio`` property

      * **_jog_joint_limit** – The maximum joint distance allowed during a jog command

      * **_trajectory_error_tol** – The maximum error allowed between command and robot position during trajectory 
        execution

      * **_command_mode** – The current command mode. Set using ``command_mode`` property, and updated during operation
        due to errors or other events.

      * **_operational_mode** – The operational mode of the vendor robot controller, using values from 
        ``RobotOperationalMode`` enum. Should be
        updated every timestep if available. Set ``_base_set_operational_mode`` to False
        if used.

      * **_controller_state** – The controller state of the vendor robot controller, using values from 
        ``RobotOperationalMode`` enum. Should be
        updated every timestep if available. Set ``_base_set_controller_state`` to False
        if used.

      * **_joint_position** – Current joint position based on feedback in radians (or meters). This value should be
        updated every timestep using robot feedback.

      * **_joint_velocity** – Current joint velocity based on feedback in radians/s (or meters/s). This value should be
        updated every timestep using robot feedback. Leave as empty array if velocity feedback
        not available.

      * **_joint_effort** – Current joint effort based on feedback in Nm (or N). This value should be
        updated every timestep using robot feedback. Leave as empty array if effort feedback
        not available.

      * **_position_command** – Current position command. Set by the subclass after issuing command to robot. This
        value is used for client state information.

      * **_velocity_command** – Current velocity command. Set by the subclass after issuing command to robot. This
        value is used for client state information.

      * **_endpoint_pose** – Array of endpoint poses, one entry per chain. Update every timestep. Units should be in 
        meters, quaternions, relative to world or base of robot.

      * **_endpoint_vel** – Array of endpoint velocities, one entry per chain. Update every timestep. Units should be in 
        meters/s, radians/s, relative to world or base of robot.

      * **_last_robot_state** – The stopwatch time in seconds of the last state update received from the robot. 
        Must be updated to avoid communication timeout.

      * **_last_joint_state** – The stopwatch time in seconds of the last joint position update received from the robot. 
        Must be updated to avoid communication timeout.

      * **_last_endpoint_state** – The stopwatch time in seconds of the last endpoint update received from the robot. 
        Must be updated to avoid communication timeout.

      * **_state_seqno** – Counter of number of loop iterations executed (sequence number)

      * **_homed** – Set to True if robot is homed. Only valid if robot has homing capability

      * **_ready** – Set to True if robot is ready to move. Should be updated every timestep

      * **_enabled** – Set to True if robot is enabled with motors on. Should be updated every timestep. Robot may
        be enabled but not ready

      * **_stopped** – Set to True if robot is stopped due to an estop. Should be updated every timestep

      * **_error** – Set to True if robot is in an error state. Should be updated every timestep. Errors are reset by
        switching to halt more, calling ``reset_errors()``, and/or clearing the error on the vendor
        controller, in escalating levels of severity.

      * **_estop_source** – The source of the estop, using values from ``RobotStateFlags``

      * **_communication_failure** – Set by ``_verify_communication`` based on ``_communication_timeout``

      * **_communication_timeout** – Communication timeout in seconds. If no updates are received from the controller
        within the communication timeout, an error condition is set

      * **_broadcast_downsampler** – Broadcast downsampler used by all wires and pipes to control data rate sent to client

      * **position_command** – Wire populated by Robot Raconteur to receive streaming position commands. Only used 
        in ``position_command`` command mode

      * **velocity_command** – Wire populated by Robot Raconteur to receive streaming position commands. Only used 
        in ``velocity_command`` command mode

      * **_wires_ready** – Set to True when wires and pipes have been initialized by Robot Raconteur

      * **_config_seqno** – The sequence number returned as part of ``RobotInfo``. Incremented as tools and payloads
        are attached/detached.

      * **_base_set_operational_mode** – If True, abstract robot will set ``_operational_mode`` to a default value.
        Set to False if driver will update ``_operational_mode``

      * **_base_set_controller_state** – If True, abstract robot will set ``_controller_state`` to a default value.
        Set to False if driver will update ``_controller_state``

   >>``<<ivar _lock: Lock to hold when updating data to prevent race conditions

   :Parameters:      
      * **robot_info** – The ``RobotInfo`` structure for the robot

      * **default_joint_count** – The default number of joints for the robot

      * **node** – The Robot Raconteur node for the driver

   **_abort_trajectory(trajectory)**

      Aborts trajectory and all trajectories by dropping to ``halt`` command made. Called by trajectory
      generater if ``Abort()`` is called.

   **_calc_endpoint_pose(chain)**

      Compute endpoint pose for specified chain. By default uses ``_endpoint_pose[chain]`` and transforms
      to the TCP of ``self._current_tool[chain]``. If the robot reports the endpoint position with the tool
      transform applied, this should return ``self._endpoint_pose[chain]``

      Called by the loop each timestep to update driver state.

      :Parameters:      
         **chain** – The chain index, always 0 for single arm driver

      :Return type:     
         com.robotraconteur.geometry.Pose

      :Returns:         
         The pose of the end effector

   **_calc_endpoint_poses()**

      Compute the endpoints of all chains. Calls ``_calc_endpoint_pose()`` for each chain.

      Called by the loop each timestep to update driver state.

      :Return type:     
         com.robotraconteur.geometry.Pose[]

      :Returns:         
         Array of all chain poses. Single element array for single arm drivers

   **_calc_endpoint_vel(chain)**

      Compute spatial velocity for specified chain. By default uses ``_endpoint_vel[chain]`` and applies TCP
      transform of ``self._current_tool[chain]``. If the robot reports the endpoint position with the tool
      transform applied, this should return ``self._endpoint_vel[chain]``

      Called by the loop each timestep to update driver state.

      :Parameters:      
         **chain** – The chain index, always 0 for single arm driver

      :Return type:     
         com.robotraconteur.geometry.SpatialVelocity

      :Returns:         
         The spatial velocity (6x1) of the end effector

   **_calc_endpoint_vels()**

      Compute the spatial velocity of all chains. Calls ``_calc_endpoint_vel()`` for each chain.

      Called by the loop each timestep to update driver state.

      :Return type:     
         com.robotraconteur.geometry.SpatialVelocity[]

      :Returns:         
         Array of all chain spatial velocities. Single element array for single arm drivers

   **_cancel_trajectory(trajectory)**

      Cancel a trajectory that is in the queue. Called from the trajectory generator if ``Close()`` is called.

   **_close()**

      Close the driver, stop the loop

   **_fill_robot_command(now)**

      Fill robot command to send to robot based on current state and commands sent by the client. Returns a
      tuple containing three elements: ``success``, ``joint_position_command``, ``joint_velocity_command``.
      If success is False, the driver cannot generate a command in its current state. If ``success`` is True,
      either ``joint_position_command`` will be non-Null, or ``joint_velocity_command`` will be non-Null.
      ``joint_velocity_command`` is only valid if the driver has the ``velocity_command`` driver capability.
      ``joint_position_command`` is in radians (or meters), while ``joint_velocity_command`` is in radians/s 
      (or meters/s)

      This function is called by the loop every timestep, and the return is passed to ``_send_joint_command()``.
      It is not typically called by the implementing class.

      :Parameters:      
         **now** – stopwatch time in seconds

      :Return type:     
         Tuple[bool,np.array,np.array]

      :Returns:         
         ``success``, ``joint_position_command``, ``joint_velocity_command``

   **_fill_state_flags(now)**

      Fill ``_robot_state_flags`` based on current state of driver. Called by the loop each timestep to update 
      driver state

      :Parameters:      
         **now** – stopwatch time in seconds

   **_fill_states(now)**

      Fill the ``RobotState``, ``AdvancedRobotState``, and ``RobotStateSensorData`` structures based on
      current driver state.

      Called by the loop each timestep to fill data to send to clients.

      :Parameters:      
         **now** – stopwatch time in seconds

      :Return type:     
         Tuple[RobotState,AdvancedRobotState,RobotStateSensorData]

   **_loop_thread_func()**

      Loop thread entry function. This function runs the loop, and calls ``run_timestep()`` periodically at
      ``_update_period`` specified in seconds.

   **_perf_counter()**

      System performance counter in seconds. This counter is not relative to real time clock.

      :Return type:     
         ``float``

      :Returns:         
         Performance counter time in seconds

   **_run_timestep(now)**

      Called by loop each timestep at ``_update_timestep`` period in seconds

      :Parameters:      
         **now** – stopwatch time in seconds

   **abstract _send_disable(handler)**

      Called to send a disable command to the robot. Only valid if driver has ``software_enable`` capability.
      Implementing class must override if used. ``handler`` must be called to complete the asynchronous request.

   **abstract _send_enable(handler)**

      Called to send an enable command to the robot. Only valid if driver has ``software_enable`` capability.
      Implementing class must override if used. ``handler`` must be called to complete the asynchronous request.

   **abstract _send_reset_errors(handler)**

      Called to send an reset errors command to the robot. Only valid if driver has ``software_reset_errors`` 
      capability. Implementing class must override if used. ``handler`` must be called to complete the asynchronous 
      request.

   **abstract _send_robot_command(now, joint_pos_cmd, joint_vel_cmd)**

      Called each timestep to send robot command. Must be implemented by subclass.

      Both ``joint_pos_cmd`` and ``joint_vel_cmd`` may be None if there is no valid command available.
      If ``joint_pos_cmd`` is non-Null, a joint position command must be sent. All drivers must support
      position command. ``joint_vel_cmd`` is only used for ``velocity_command`` mode, and is only supported
      if the driver has ``velocity_command`` capability.

      :Parameters:      
         * **now** – stopwatch time in seconds

         * **joint_pos_cmd** – Joint position command in radians (or meters)

         * **joint_vel_cmd** – Joint velocity command in radians/s (or meters/s)

   **_send_states(now, rr_robot_state, rr_advanced_robot_state, rr_state_sensor_data)**

      Sends the states to the Robot Raconteur clients using broadcast wires

      Called by the loop each timestep to send data to clients.

      :Parameters:      
         * **now** – stopwatch time in seconds

         * **rr_robot_state** – populated RobotState instance

         * **rr_advanced_robot_state** – populated AdvancedRobotState instance

         * **rr_state_sensor_data** – populated RobotStateSensorData instance

   **_start_robot()**

      Start the robot driver loop

   **_stop_robot()**

      Stop the robot driver loop

   **_stopwatch_ellapsed_s()**

      Stopwatch time in seconds. Relative to start of driver loop.

      :Return type:     
         ``float``

      :Returns:         
         Stopwatch time in seconds

   **_verify_communication(now)**

      Verify that the driver is communicating with robot. Compares last communication tomi te 
      ``_communication_timeout`` to determine when communication has been lost.

      Called by the loop each timestep to check if robot is still communicating.

      :Parameters:      
         **now** – stopwatch time in seconds

   **_verify_robot_state(now)**

      Verify that the robot is ready to operate, or if an error has occurred. Drops to ``halt`` command mode
      if robot is not ready. Drops to ``error`` command mode if error has occurred.

      :Parameters:      
         **now** – stopwatch time in seconds

   **async_disable(handler)**

      Called by client to request robot disable. Calls ``_send_disable()``

   **async_enable(handler)**

      Called by client to request robot enable. Calls ``_send_enable()``

   **async_getf_signal(signal_name, handler)**

      Get the value of a signal. Optionally implemented by subclass

   **async_home(handler)**

      Called by client to home the robot. Behavior is device specific.

      Robot must be in ``homing`` command mode to call this function.

      :Parameters:      
         **handler** (*Callable**[**[**]**,**Exception**]*) – Handler to call when function is complete

   **async_jog_freespace(joint_position, max_velocity, wait, handler)**

      Called by client to jog the robot to a specified joint position with specified maximum joint velocity. If wait
      is True, the function will not return to the client until the move is complete. Otherwise will return
      immediately.

      This function is typically used to jog the robot to a specific position.

      Robot must be in ``jog`` command mode to call this function.

      This is an asynchronous function, and handler must be called to return result to the client.

      :Parameters:      
         * **joint_position** (*np.ndarray*) – The desired joint position in radians

         * **max_velocity** (*np.ndarray*) – The maximum joint velocity in radians/s

         * **wait** (*bool*) – Wait for completion or return immediately

         * **handler** (*Callable**[**[**]**,**Exception**]*) – Handler to call when function is complete

   **async_jog_joint(joint_velocity, timeout, wait, handler)**

      Called by client to jog the robot at a specified joint velocity for a specified time. If wait
      is True, the function will not return to the client until the move is complete. Otherwise will return
      immediately.

      This function is typically called repeatedly by the client (with wait=False) to drive the robot in response to
      user input such as a panel button or joystick.

      Robot must be in ``jog`` command mode to call this function.

      This is an asynchronous function, and handler must be called to return result to the client.

      :Parameters:      
         * **joint_velocity** – The desired joint velocity position in radians/s

         * **timeout** (*float*) – The timeout to run at the specified velocity

         * **wait** (*bool*) – Wait for completion or return immediately

         * **handler** (*Callable**[**[**]**,**Exception**]*) – Handler to call when function is complete

   **async_reset_errors(handler)**

      Called by client to request software reset errors. Calls ``_send_reset_errors()``

   **async_setf_signal(signal_name, value, handler)**

      Set the value of a signal. Optionally implemented by subclass

   ``property command_mode``

      Returns the current ``command_mode``

   **controller_state()**

      Return the current state of the vendor robot controller, if available

   **current_errors()**

      Returns currently reported errors, if available

   ``property device_info``

      Returns the DeviceInfo structure contained in RobotInfo

   **execute_trajectory(trajectory)**

      Called by the client to execute a trajectory. Must be in ``trajectory`` command mode.

      This function returns a generator. The client must call ``Next()`` repeatedly on the generator
      until the trajectory is complete.

      The first waypoint on the trajectory must be reasonably close to the current robot position.

      :Parameters:      
         **trajectory** (*JointTrajectory*) – The trajectory to execute

      :Returns:         
         The trajectory generator, that must have ``Next()`` called repeatedly to execute trajectory

      :Return type:     
         TrajectoryStatus{generator}

   **getf_param(param_name)**

      Get the value of a parameter. Optionally implemented by subclass

   ``property isoch_downsample``

      Return the current client isoch_downsample level

   ``property isoch_info``

      Returns the IsochInfo structure

   **jog_cartesian(velocity, timeout, wait)**

      Called by client to jog the robot at a specified cartesian velocity for a specified time. If wait
      is True, the function will not return to the client until the move is complete. Otherwise will return
      immediately.

      This function is typically called repeatedly by the client (with wait=False) to drive the robot in response to
      user input such as a panel button or joystick.

      Robot must be in ``jog`` command mode to call this function.

      This is an asynchronous function, and handler must be called to return result to the client.

      :Parameters:      
         * **velocity** – The desired end effector spatial velocity position in meters/s,radians/s

         * **timeout** (*float*) – The timeout to run at the specified velocity

         * **wait** (*bool*) – Wait for completion or return immediately

         * **handler** (*Callable**[**[**]**,**Exception**]*) – Handler to call when function is complete

   ``property operational_mode``

      Return the current operational mode of the controller, if available

   **payload_attached(chain, payload, pose)**

      Called by client to notify the driver that a payload has been attached to the tool. A tool must be attached
      to attach a payload. The pose between the payload and tool is also specified.

      Implementing class may also update the vendor robot controller if necessary.

      :Parameters:      
         * **chain** (*int*) – The kinematic chain the tool has been attached

         * **payload** – The PayloadInfo structure of the tool, specified by the client

         * **pose** (*com.geometry.Pose*) – The pose of the payload relative to the tool TCP

   **payload_detached(chain, payload_name)**

      Called by client to notify the driver that a payload has been detached

      :Parameters:      
         **payload_name** (*str*) – The name of the payload that was detached

   ``property robot_info``

      Returns the current ``RobotInfo`` structure. The ``RobotInfo`` structure will be updated with tool
      and payload information as it changes.

      :Returns:         
         The populated RobotInfo structure

      :Return type:     
         RobotInfo

   **setf_param(param_name, value)**

      Set the value of a parameter. Optionally implemented by subclass

   ``property speed_ratio``

      Get the speed ratio

   **tool_attached(chain, tool)**

      Called by client to notify the driver that a tool has been attached. TCP is used to compute endpoint position
      and velocity. Implementing class may also update the vendor robot controller if necessary.

      :Parameters:      
         * **chain** (*int*) – The kinematic chain the tool has been attached

         * **tool** (*ToolInfo*) – The ToolInfo structure of the tool, specified by the client

   **tool_detached(chain, tool_name)**

      Called by client to notify the driver that a tool has been detached. Payloads must be detached before
      the tool can be detached.

      :Parameters:      
         **payload_name** (*str*) – The name of the tool that was detached
