from operator import truediv
from matplotlib.pyplot import close
from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
# import keyboard

from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from collections import deque
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner

class PIDCurveController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_radius = 1000
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        
        # useful variables
        self.region = 1
        self.brake_counter = 0

        self.waypoint_queue_region = []
        with open("ROAR\\control_module\\region_list.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_region.append(waypoint)

        self.waypoint_queue_braking = []
        with open("ROAR\\control_module\\braking_list_mod.txt") as f:
            for line in f:
                raw = line.split(",")
                waypoint = Transform(location=Location(x=raw[0], y=raw[1], z=raw[2]), rotation=Rotation(pitch=0, yaw=0, roll=0))
                self.waypoint_queue_braking.append(waypoint)

        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def run_in_series(self, 
                      next_waypoint: Transform, 
                      close_waypoint: Transform, 
                      far_waypoint: Transform, 
                      more_waypoints: [Transform], **kwargs) -> VehicleControl:

        # run lat pid controller
        steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        
        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        #print(error, wide_error, sharp_error)

        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])

        if self.region == 1:
            throttle, brake = self._get_throttle_and_brake(more_waypoints)
        elif self.region == 2:
            waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                self.brake_counter += 1
                if self.brake_counter >= 4:
                    self.brake_counter = 0
            elif dist <= 15:
                self.brake_counter = 1
                throttle = -1
                brake = 1
                # print(self.waypoint_queue_braking[0])
                self.waypoint_queue_braking.pop(0)
            else:
                throttle, brake = self._get_throttle_and_brake(more_waypoints)

            # elif sharp_error >= 0.67 and current_speed > 70:
            #     throttle = 0
            #     brake = 0.4
            # elif wide_error > 0.09 and current_speed > 92: # wide turn
            #     throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
            #     brake = 0
            # else:
            #     throttle = 1
            #     brake = 0
        
        gear = 1
        # gear = max(1, (int)((current_speed - 2*pitch) / 60))
        # if throttle == -1:
        #     gear = -1
        
        waypoint = self.waypoint_queue_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist <= 10:
            self.region += 1
            self.waypoint_queue_region.pop(0)
        
        # if keyboard.is_pressed("space"):
        #      print(self.agent.vehicle.transform.record())
        
        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)

    def _get_throttle_and_brake(self, more_waypoints: [Transform]):
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        wp = self._get_next_interesting_waypoints(more_waypoints, current_speed)
        r1 = self._get_radius(wp[0:3])
        r2 = self._get_radius(wp[2:5])
        target_speed1 = self._get_target_speed(r1)
        target_speed2 = self._get_target_speed(r2)
        t, b = self._get_throttle_updates(current_speed, target_speed1, target_speed2)
        if t < 1.0:
            print("\n\nART: got radius " + str(r1) + " " + str(r2) 
                + " tspeed1= " + str(target_speed1) 
                + " tspeed2= " + str(target_speed2) 
                + " cspeed= " +  (str(current_speed)) 
                + " t= " + str(t) + " b= " + str(b)
                #   + " wp= " + str(wp)
                )
        return t, b
    
    def _get_next_interesting_waypoints(self, more_waypoints: [Transform], current_speed):
        target_distance = [50, 100, 150, 200]
        # if current_speed < 50:
        #     target_distance = [30, 60, 110, 160]
        points = []
        start = self.agent.vehicle.transform
        points.append(start)
        curr_dist = 0
        num_points = 0
        for p in more_waypoints:
            end = p
            num_points += 1
            curr_dist += start.location.distance(end.location)
            if curr_dist > target_distance[len(points) - 1]:
                points.append(end)
            start = end
            if len(points) > len(target_distance):
                break

        # print("\n\nART: using num_points= " + str(len(points)))
        return points

    def _get_radius(self, wp: [Transform]):
        point1 = (wp[0].location.x, wp[0].location.z)
        point2 = (wp[1].location.x, wp[1].location.z)
        point3 = (wp[2].location.x, wp[2].location.z)

        # Calculating length of all three sides
        len_side_1 = round( math.dist(point1, point2), 3)
        len_side_2 = round( math.dist(point2, point3), 3)
        len_side_3 = round( math.dist(point1, point3), 3)
        small_num = 0.01
        if len_side_1 < small_num or len_side_2 < small_num or len_side_3 < small_num:
            return self.max_radius

        # sp is semi-perimeter
        sp = (len_side_1 + len_side_2 + len_side_3) / 2

        # Calculating area using Herons formula
        area_squared = sp * (sp - len_side_1) * (sp - len_side_2) * (sp - len_side_3)
        if area_squared < small_num:
            return self.max_radius
        # Calculating curvature using Menger curvature formula
        radius = (len_side_1 * len_side_2 * len_side_3) / (4 * math.sqrt(area_squared))

        return radius
        
    def _get_target_speed(self, radius):
        if radius >= self.max_radius:
            return self.max_speed
        mu = 0.6
        return int(math.sqrt(mu*9.81*radius) * 3.6)

    def _get_throttle_updates(self, current_speed, target_speed1, target_speed2):
        throttle = 0.9
        brake = 0
        if (current_speed + 2 < target_speed1):
            throttle = 1
            brake = 0
        if (current_speed < target_speed1) and (current_speed < target_speed2):
            throttle = 1
            brake = 0
        if current_speed > target_speed1:
            throttle = -1
            brake = 1
        if current_speed > target_speed2 and current_speed - target_speed2 > 30:
            throttle = -1
            brake = 1

        return throttle, brake

    @staticmethod
    def find_k_values(vehicle: Vehicle, config: dict) -> np.array:
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])

class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #error = np.arccos(v_vec_normed @ w_vec_normed.T)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        _cross = np.cross(v_vec_normed, w_vec_normed)

        # calculate close error projection
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        # calculate far error projection
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        if _cross[1] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = PIDCurveController.find_k_values(config=self.config, vehicle=self.agent.vehicle)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
        return lat_control, error, wide_error, sharp_error