# Bridges
## bridges
### Bridges.bridge
- `C` This file defines a basic Bridge for extensibility of the ROAR Autonomous software
## carla_bridge
### Bridges.carla_bridge
- `C` CarlaBridge
- `F` convert_location_from_source_to_agent: 

Convert Location data from Carla.location to Agent's location data type

invert the Z axis to make it into right hand coordinate system

Args:

	source: carla.location

Returns:

	None

- `F` convert_rotation_from_source_to_agent:

Convert a CARLA raw rotation to Rotation(pitch=float,yaw=float,roll=float).

- `F` convert_transform_from_source_to_agent:

Convert CARLA raw location and rotation to Transform(location,rotation).

- `F` convert_control_from_source_to_agent:

Convert CARLA raw vehicle control to VehicleControl(throttle,steering).

- `F` convert_rgb_from_source_to_agent:

Convert CARLA raw Image to a Union with RGB numpy array.

- `F` convert_depth_from_source_to_agent:

Convert CARLA raw Image to a Union with Depth numpy array.

- `F` convert_vector3d_from_source_to_agent:

Convert CARLA raw Vector3d Data to a Vector3D Object.

- `F` convert_imu_from_source_to_agent:

Convert CARLA raw IMUData to IMUData(accelerometer, gyroscope).

- `F` convert_sensor_data_from_source_to_agent:

Returns CARLA Sensors Data from raw front RGB, rear RGB, front depth, and IMU Data.

- `F` convert_lidar_from_source_to_agent:

Get the lidar data and convert it to a numpy array.

- `F` convert_vehicle_from_source_to_agent:

Converts Velocity, Transform, and Control of carla.Vehicle

- `F` convert_control_from_agent_to_source:

Converts control to carla.VehicleControl

- `F` convert_vector3d_from_agent_to_source:

Convert Vector3D Object to a CARLA raw Vector3d Data.

- `F` convert_location_from_agent_to_source:

Convert Agent's Location to a Carla.Location.

- `F` convert_rotation_from_agent_to_source:

Convert Agent's Rotation to a Carla.Rotation.

- `F` convert_transform_from_agent_to_source:

Convert Agent's Transform to a Carla.Transform.

- `F` _to_bgra_array:

Convert a CARLA raw image to a BGRA numpy array.

- `F` _to_rgb_array:

Convert a CARLA raw image to a RGB numpy array.

## jetson_bridge

### Bridges.jetson_bridge

- `C` JetsonBridge

- `F` convert_location_from_source_to_agent:

Convert Location data from Jetson Vehicle to Agent's location data type.

Args:

	source ():

Returns:

	Location(x, y, z)

- `F` convert_rotation_from_source_to_agent:

Convert a Jetson raw rotation to Rotation(pitch=float,yaw=float,roll=float).

Args:

	source ():

Returns:

	Rotation(pitch, yaw, roll)

- `F` convert_transform_from_source_to_agent:

Convert Jetson raw location and rotation to Transform(location,rotation).

Args:

	source ():

Returns:

	Transform(Location, Rotation)

- `F` convert_control_from_source_to_agent:

Convert Jetson raw vehicle control to VehicleControl(throttle,steering).

Args:

	source ():

Returns:

	VehicleControl(Throttle, Steering)

- `F` convert_rgb_from_source_to_agent:

Convert Jetson raw Image to an Optional with RGB numpy array.

Args:

	source ():

Returns:

	RGBData

- `F` convert_depth_from_source_to_agent:

Convert Jetson raw Image to an Optional with Depth numpy array.

Args:

	source ():

Returns:

	DepthData

- `F` convert_vector3d_from_source_to_agent:

Convert Jetson raw Vector3d Data to a Vector3D Object.

Args:

	source ():

Returns:

	Vector3D(x, y, z)

- `F` convert_imu_from_source_to_agent:

Convert Jetson raw IMUData to IMUData(accelerometer, gyroscope).

Args:

	source ():

Returns:

	IMUData(accelerometer, gyroscope)

- `F` convert_sensor_data_from_source_to_agent:

Returns Jetson Sensors Data from raw front RGB, rear RGB, front depth, and IMU Data.

Args:

	source ():

Returns:

	SensorData(front_RGB, rear_RGB, front_depth, IMU_Data)

- `F` convert_vive_tracker_data_from_source_to_agent:

Converts raw Vive Tracker data to ViveTrackerData(Location, Rotation, Velocity).

Args:

	source ():

Returns:

	ViveTrackerData(Location, Rotation, Velocity)

- `F` convert_vehicle_from_source_to_agent:

Converts Transform, Velocity, Wheel_Base, and Control of JetsonVehicle.

Args:

	source ():

Returns:

	Vehicle(Transform, Velocity, Wheel_Base, Control)

- `F` convert_control_from_agent_to_source:

Converts control to Throttle and Steering numpy arrays bound between -1 and 1.

Args:

	control ():

Returns:

	Tuple

- `F` convert_vector3d_from_agent_to_source:

Convert Jetson Vector3D Object to Vector3D data.

Args:

	vector3d ():

Returns:

	Array



# ROAR

## agent_module

### ROAR.agent_module.forward_only_agent

- `C` ForwardOnlyAgent

- `F` run_step

### ROAR.agent_module.opencv_tensorflow_object_detection_agent

- `C` OpenCVTensorflowObjectDetectionAgent

- `F` run_step

### ROAR.agent_module.pid_agent

- `C` PIDAgent

- `F` run_step

### ROAR.agent_module.pure_pursuit_agent

- `C` PurePursuitAgent

- `F` run_step

### ROAR.agent_module.lqr_agent

- `C` LQRAgent

- `F` run_step

## config

### ROAR.configurations.configuration

- `C` Configuration

## controller_module

### ROAR.control_module.controller

- `C` Controller

- `F` run_in_series:

Abstract function for run step

Args:

	next_waypoint: next waypoint

	**kwargs:

Returns:

	VehicleControl

### ROAR.control_module.lqr_controller

- `C` LQRController

- `F` _dlqr:

solves the infinite-horizon discrete-time lqr

- `F` run_in_series

- `F` _calculate_angle_error:

code stolen from the PID controller to calculate the angle

### ROAR.control_module.mpc_controller

- `C` _EqualityConstraints:

Class for storing equality constraints in the MPC.

- `C` VehicleMPCController

- `F` run_in_series

- `F` get_func_constraints_and_bounds:

Defines MPC's cost function and constraints.

- `F` generate_fun:

Generates a function of the form `fun(x, *args)

- `F` generate_grad

- `F` get_state0

- `F` minimize_cost

- `F` get_closest_waypoint_index_3D:

Get the index of the closest waypoint in self.track_DF

car_location: current car location

waypoint_location: next_waypoint

- `F` get_closest_waypoint_index_2D:

Get the index of the closest waypoint in self.pts_2D

Note: it may give wrong index when the route is overlapped

- `F` create_array_of_symbols

- `F` transform_into_cars_coordinate_system

- `F` clip_throttle

### ROAR.control_module.pid_controller

- `C` PIDController

- `F` run_in_series

- `F` find_k_values

- `C` LongPIDController

- `F` run_in_series

- `C` LatPIDController

- `F` run_in_series:

Calculates a vector that represent where you are going.

Args:

	next_waypoint ():

	**kwargs ():

Returns:

	lat_control

### ROAR.control_module.pure_pursuit_control

- `C` PurePursuitController:

Args:

	vehicle: Vehicle information

	look_ahead_gain: Look ahead factor

	look_ahead_distance: look ahead distance

	target_speed: desired longitudinal speed to maintain

- `F` run_in_series:

run one step of Pure Pursuit Control

Args:

	vehicle: current vehicle state

	next_waypoint: Next waypoint, Transform

	**kwargs:

Returns:

	Vehicle Control

- `C` LongitunalPurePursuitController

- `C` LatitunalPurePursuitController

- `F` run_step

Args:

	next_waypoint ():

Returns:

	VehicleControl.clamp

### ROAR.control_module.rl_pid_controller

- `C` PIDController

- `F` run_in_series

- `C` LongPIDController

- `F` run_in_series

- `F` find_k_values

- `C` LatPIDController

- `F` run_in_series

calculate a vector that represent where you are going

- `F` find_k_values

## perception_module

### ROAR.perception_module.depth_to_pointcloud_detector

- `C` DepthToPointCloudDetector

- `F` run_in_threaded

- `F` run_in_series:

return: 3 x N array of point cloud

- `F` pcd_via_open3d

- `F` old_way

- `F` _pix2xyz

- `C` DepthToPCDConfiguration

### ROAR.perception_module.detector

- `C` Detector

### ROAR.perception_module.ground_plane_detector

- `C` GroundPlaneDetector

- `F` run_in_series

- `F` construct_pointcloud

- `F` compute_reference_norm

- `F` normalize_v3

- `F` compute_vectors_near_me

## planning_module

### ROAR.abstract_planner

- `C` AbstractPlanner

- `F` run_in_series:

On every step, produce an actionable plan

Returns: None

## utilities_module

### ROAR.utilities

- `F` get_ip

- `F` png_to_depth:

Takes in an image read from cv2.imread(), whose output_oct_10 is simply a numpy array, turn it into a depth image according to carla's method of (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).

Args:

	im: input image, read from cv2.imread()

Returns:

	depth image

- `F` img_to_world:

Compute image to world translation using the formula below

((R_world_veh)^(-1) @ (R_veh_cam)^(-1) @ ((intrinsics)^(-1) @ scaled_depth_image).pad_with_1)[:3, :] = [X Y Z]

Args:

	scaled_depth_image: 3 x n numpy array

	intrinsics_matrix: 3 x 3 intrinsics

	veh_world_matrix: 4 x 4 vehicle to world transformation matrix

	cam_veh_matrix: 4 x 4 camera to vehicle transformation matrix

Returns:

	n x 3 array of n points

- `F` img_to_world2

- `F` rotation_matrix_from_euler:

Takes in roll pitch yaw and compute rotation matrix using the order of

R = R_yaw * R_pitch * R_roll

http://planning.cs.uiuc.edu/node104.html

Args:

	roll: float of roll in degree

	pitch: float of pitch in degree

	yaw: float of yaw in degree

Returns:

	3 x 3 array rotation matrix

## visualization

### ROAR.visualizer

- `C` Visualizer

- `F` world_to_img_transform

Calculate the 2D image coordinate from 3D world space

Args:

	xyz: (Nx3) array representing X, Y, Z in world coord

Returns:

	Array if integers [u, v, f]

- `F` show_first_person_visualization

Visualizes image from Front RGB Camera.

Args:

	show_num_waypoints ():

	show_semantic_segmentation_obstacle ():

	show_semantic_segmentation_sky ():

	show_semantic_segmentation_ground ():

	show_point_cloud_ground ():

	ground_points ():

Returns:

	None

- `F` show_birds_eye_visualization

Visualizes top down image of Agent.

Args:

	focus_on_vehicle ():

	view_size ():

Returns:

	None

# ROAR_Sim

### ROAR_Sim.carla_client.util.camera_manager

- `C` CameraManager

- `F` toggle_camera

- `F` set_sensor

- `F` next_sensor

- `F` toggle_recording

- `F` render

- `F` _parse_image

### ROAR_Sim.configurations.configuration

- `F` import_carla

- `C` Configuration

### ROAR_Sim.carla_client.util.hud

- `C` HUD

- `F` on_world_tick

- `F` tick

- `F` toggle_info

- `F` notification

- `F` error

- `F` render

- `C` FadingText

- `F` set_text

- `F` tick

- `F` render

### ROAR_Sim.carla_client.util.keyboard_control

- `C` KeyboardControl:

Class that handles keyboard input.

- `F` parse_events:

Parse keyboard press.

:param client: carla.Client

:param world: carla.Client

:param clock: pygame clock

:return:

	bool - True if should continue, aka no exit key was pressed

	control - carla.VehicleControl

- `F` _parse_joystick

- `F` _parse_vehicle_keys

- `F` _parse_walker_keys

- `F` _is_quit_shortcut

### ROAR_Sim.carla_client.util.sensors

- `C` CollisionSensor

- `F` get_collision_history

- `F` _on_collision

- `C` GnssSensor

- `F` _on_gnss_event

- `C` IMUSensor

- `F` _IMU_callback

- `C` LaneInvasionSensor

- `F` _on_invasion

- `C` RadarSensor

- `F` _Radar_callback

### ROAR_Sim.carla_client.util.utilities

- `F` get_actor_display_name

- `F` create_dir_if_not_exist

- `C` CarlaCarColor

- `C` CarlaCarColors

- `C` CarlaWeather:

Default weather is sunny

- `C` CarlaWeathers

### ROAR_Sim.carla_client.util.world

- `C` World:

A World that holds all display settings, Create a World with the given carla_world, head-up display and server setting.

- `F` spawn_actor:

Set up a hero-named player with Grey Tesla Model3 Vehicle

- `F` set_camera

- `F` set_sensor

- `F` toggle_radar

- `F` tick

- `F` render

- `F` destroy_sensors

- `F` set_weather

- `F` set_custom_sensor

- `F` _spawn_custom_sensor

- `F` _destroy_custom_sensors

- `F` _parse_front_rgb_sensor_image

- `F` _parse_front_depth_sensor_image

- `F` _parse_lidar_sensor_data

- `F` _parse_rear_rgb_sensor_image

- `F` _parse_semantic_segmentation_image

- `F` spawn_npcs

- `F` destroy

- `F` clean_spawned_all_actors:

This function is to clean all actors that are not traffic light/signals
