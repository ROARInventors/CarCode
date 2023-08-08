from operator import truediv
from matplotlib.pyplot import close
from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
# import keyboard

from ROAR.utilities_module.data_structures_models import Transform, Location, Rotation
from collections import deque
from enum import Enum
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner

class SpeedData:
    def __init__(self, distance_to_section, current_speed, target_speed, recommended_speed):
        super().__init__()
        self.current_speed = current_speed
        self.distance_to_section = distance_to_section
        self.target_speed_at_distance = target_speed
        self.recommended_speed_now = recommended_speed
        self.speed_diff = current_speed - recommended_speed

class PIDFastController(Controller):
    # save debug messages to show after crash or finish.
    display_debug = False
    debug_strings = deque(maxlen=1000)

    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_radius = 10000
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        self.intended_target_distance = [0, 30, 60, 90, 120, 150] 
        self.target_distance = [0, 30, 60, 90, 120, 150]
        self.close_index = 0
        self.mid_index = 1
        self.far_index = 2
        self.tick_counter = 0
        self.previous_speed = 1.0
        self.brake_counter = 0
        self.forced_brake_counter = 0
        self.brake_ticks = 0
        self.region = 1

        # for testing how fast the car stops
        self.brake_test_counter = 0
        self.brake_test_in_progress = False

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
            dt=0.05,
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def __del__(self):
        for s in self.__class__.debug_strings:
            print(s)

    # modified
    def run_in_series(self, 
                      next_waypoint: Transform, 
                      close_waypoint: Transform,  # ok to remove 
                      far_waypoint: Transform,    # ok to remove
                      more_waypoints: [Transform], **kwargs) -> VehicleControl:

        # run lat pid controller
        # NOTE: ok to remove wide_error and sharp_error
        steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        
        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        #print(error, wide_error, sharp_error)

        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])

        self.tick_counter += 1
        waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        break_for_counts = self._get_forced_brake_counter_for_waypoint(waypoint)
        if dist <= 5 and break_for_counts > 0:
            self.forced_brake_counter = break_for_counts - 1
            throttle = -1
            brake = 0.8
            print("\nspecial break point: ")
            print(self.waypoint_queue_braking[0])
            self.waypoint_queue_braking.pop(0)
        elif self.forced_brake_counter > 0:
            throttle = -1
            brake = 1
            self.forced_brake_counter -= 1
        else:
            throttle, brake = self._get_throttle_and_brake(more_waypoints, wide_error)

        # gear = 1        
        gear = max(1, (int)((current_speed - 2*pitch) / 60))
        if throttle == -1:
            gear = -1
        
        waypoint = self.waypoint_queue_region[0]
        ## region related stuff is ok to remove
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist <= 10:
            self.region += 1
            self.waypoint_queue_region.pop(0)

        #throttle, brake = self._brake_test(throttle, brake)
        
        # if keyboard.is_pressed("space"):
        #      print(self.agent.vehicle.transform.record())

        self.dprint("--- " + str(throttle) + " " + str(brake) 
                    + " steer " + str(steering)
                    + "     loc x,z" + str(self.agent.vehicle.transform.location.x)
                    + " " + str(self.agent.vehicle.transform.location.z)) 

        self.previous_speed = current_speed
        if self.brake_ticks > 0 and brake > 0:
            self.brake_ticks -= 1

        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)

    def _get_forced_brake_counter_for_waypoint(self, waypoint):
        waypoint_x = int(waypoint.location.x)
        if  waypoint_x == 3017: # 
            return 2
        if  waypoint_x == 3607: # 
            return 1
        if  waypoint_x == 3695: # 
            return 1
        if  waypoint_x == 4441: # needs 3
            return 3
        if waypoint_x == 5624:
            return 5
        if waypoint_x == 4203: # 1 is enough?, maybe 2 to be on the safe side
            return 2
        if waypoint_x == 5012:  # needs 3
            return 3
        if waypoint_x == 5008:
            return 0 # todo: set to 0
        if waypoint_x == 5004:
            return 0 # todo: set to 0
        if waypoint_x == 4915:
            return 1
        return 3

    def _get_throttle_and_brake(self, more_waypoints: [Transform], wide_error):
        current_speed = Vehicle.get_speed(self.agent.vehicle)

        wp = self._get_next_interesting_waypoints(more_waypoints, current_speed)
        r1 = self._get_radius(wp[self.close_index : self.close_index + 3])
        r2 = self._get_radius(wp[self.mid_index : self.mid_index + 3])
        r3 = self._get_radius(wp[self.far_index : self.far_index + 3])

        p1 = \
            math.asin( np.clip((wp[1].location.y - wp[0].location.y) / (self.target_distance[1] - self.target_distance[0]), -0.5, 0.5))
        p2 = \
            math.asin( np.clip((wp[2].location.y - wp[0].location.y) / (self.target_distance[2] - self.target_distance[0]), -0.5, 0.5))
        # taking the steeper one, to avoid underestimating pitch on downhill.
        pitch_to_next_point = min(p1, p2)

        target_speed1 = self._get_target_speed(r1, pitch_to_next_point)
        target_speed2 = self._get_target_speed(r2, pitch_to_next_point)
        target_speed3 = self._get_target_speed(r3, pitch_to_next_point)

        close_distance = self.target_distance[self.close_index] + 3
        mid_distance = self.target_distance[self.mid_index]
        far_distance = self.target_distance[self.far_index]
        speed_data = []
        speed_data.append(self._speed_for_turn(close_distance, target_speed1, pitch_to_next_point))
        speed_data.append(self._speed_for_turn(mid_distance, target_speed2, pitch_to_next_point))
        speed_data.append(self._speed_for_turn(far_distance, target_speed3, pitch_to_next_point))
        if current_speed > 220:
            # at high speed look further ahead
            r4 = self._get_radius([wp[self.close_index], wp[self.close_index+2], wp[self.close_index+4]])
            target_speed4 = self._get_target_speed(r4, pitch_to_next_point)
            speed_data.append(self._speed_for_turn(close_distance, target_speed4, pitch_to_next_point))

        update = self._select_speed(speed_data)

        t, b = self._speed_data_to_throttle_and_brake(update, pitch_to_next_point)
        return t, b

    def _speed_data_to_throttle_and_brake(self, speed_data: SpeedData, pitch_to_next_point):
        percent_of_max = speed_data.current_speed / speed_data.recommended_speed_now

        self.dprint("dist=" + str(round(speed_data.distance_to_section)) + " cs=" + str(round(speed_data.current_speed, 2)) 
                    + " ts= " + str(round(speed_data.target_speed_at_distance, 2)) 
                    + " maxs= " + str(round(speed_data.recommended_speed_now, 2)) + " pcnt= " + str(round(percent_of_max, 2)))

        percent_change_per_tick = 0.07 # speed drop for one time-tick of braking
        speed_up_threshold = 0.99
        throttle_decrease_multiple = 0.7
        throttle_increase_multiple = 1.25
        debug_note = ""
        if pitch_to_next_point < math.radians(-6):
            percent_change_per_tick = 0.05
            speed_up_threshold = 0.94
            throttle_decrease_multiple = 0.4
            throttle_increase_multiple = 1.05
            debug_note += "-6"
            # print("changing multiples-5 " + str(pitch_to_next_point) + " deg " + str(math.degrees(pitch_to_next_point)) 
            #       + " loc " + str(self.agent.vehicle.transform.location.x))
        if pitch_to_next_point < math.radians(-8):
            percent_change_per_tick = 0.03
            speed_up_threshold = 0.85
            throttle_decrease_multiple = 0.2
            throttle_increase_multiple = 1
            debug_note += "-8"
            # print("changing multiples-6 " + str(pitch_to_next_point) + " deg " + str(math.degrees(pitch_to_next_point)) 
            #       + " loc " + str(self.agent.vehicle.transform.location.x))
        if pitch_to_next_point < math.radians(-10):
            percent_change_per_tick = 0.01
            speed_up_threshold = 0.8
            throttle_decrease_multiple = 0.2
            throttle_increase_multiple = 1
            debug_note += "-10"
        if debug_note != "":
            self.dprint("changing multiples- " + str(debug_note) + " " + str(pitch_to_next_point) + " deg " + str(math.degrees(pitch_to_next_point)) 
                    + " loc " + str(self.agent.vehicle.transform.location.x))

        percent_speed_change = (speed_data.current_speed - self.previous_speed) / (self.previous_speed + 0.0001) # avoid division by zero

        if percent_of_max > 1:
            # Consider slowing down
            if percent_of_max > 1 + percent_change_per_tick:
                if self.brake_ticks > 0:
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: counter" + str(self.brake_ticks))
                    return -1, 1
                # if speed is not decreasing fast, hit the brake.
                if self.brake_ticks <= 0 and not self._speed_dropping_fast(percent_change_per_tick):
                # if self.brake_ticks <= 0 and percent_speed_change > (-percent_change_per_tick / 2):
                    # start braking, and set for how many tick to brake
                    self.brake_ticks = math.floor((percent_of_max - 1) / percent_change_per_tick)
                    # TODO: try 
                    # self.brake_ticks = 1, or (1 or 2 but not more)
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: initiate counter" + str(self.brake_ticks))
                    return -1, 1
                else:
                    # speed is already dropping fast, ok to throttle because the effect of throttle is delayed
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle early1: sp_ch=" + str(percent_speed_change))
                    return 1, 0
            else:
                if self._speed_dropping_fast(percent_change_per_tick):
                    # speed is already dropping fast, ok to throttle because the effect of throttle is delayed
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle early2: sp_ch=" + str(percent_speed_change))
                    return 1, 0
                throttle_to_maintain = self._get_throttle_to_maintain_speed(speed_data.current_speed, pitch_to_next_point)
                if percent_of_max > 1.02 or percent_speed_change > (-percent_change_per_tick / 2):
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle down: sp_ch=" + str(percent_speed_change))
                    return throttle_to_maintain * throttle_decrease_multiple, 0 # coast, to slow down
                else:
                    self.dprint("tb: tick" + str(self.tick_counter) + " brake: throttle maintain: sp_ch=" + str(percent_speed_change))
                    return throttle_to_maintain, 0
        else:
            self.brake_ticks = 0 # done slowing down. clear brake_ticks
            # Consider speeding up
            if self._speed_dropping_fast(percent_change_per_tick):
                # speed is dropping fast, ok to throttle because the effect of throttle is delayed
                self.dprint("tb: tick" + str(self.tick_counter) + " throttle: full speed drop: sp_ch=" + str(percent_speed_change))
                return 1, 0
            if percent_of_max < speed_up_threshold:
                self.dprint("tb: tick" + str(self.tick_counter) + " throttle full: p_max=" + str(percent_of_max))
                return 1, 0
            throttle_to_maintain = self._get_throttle_to_maintain_speed(speed_data.current_speed, pitch_to_next_point)
            if percent_of_max < 0.98 or percent_speed_change < -0.01:
                self.dprint("tb: tick" + str(self.tick_counter) + " throttle up: sp_ch=" + str(percent_speed_change))
                return throttle_to_maintain * throttle_increase_multiple, 0 
            else:
                self.dprint("tb: tick" + str(self.tick_counter) + " throttle maintain: sp_ch=" + str(percent_speed_change))
                return throttle_to_maintain, 0

    # used to detect when speed is dropping due to brakes applied earlier. speed delta has a steep negative slope.
    def _speed_dropping_fast(self, percent_change_per_tick):
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        percent_speed_change = (current_speed - self.previous_speed) / (self.previous_speed + 0.0001) # avoid division by zero
        return percent_speed_change < (-percent_change_per_tick / 2)

    # find speed_data with smallest recommended speed (same as the largest speed excess [current > recommended])
    # TODO: change to look for smallest recommended.
    def _select_speed(self, speed_data: [SpeedData]):
        largest_diff = -300
        index_of_largest_diff = -1
        for i, sd in enumerate(speed_data):
            if sd.speed_diff > largest_diff:
                largest_diff = sd.speed_diff
                index_of_largest_diff = i

        if index_of_largest_diff != -1:
            return speed_data[index_of_largest_diff]
        else:
            return speed_data[0]
    
    def _get_throttle_to_maintain_speed(self, current_speed, pitch_to_next_point):
        # TODO: commpute throttle needed to maintain current speed with given pitch.
        #       need to consider current_speed
        throttle = 0.6 + current_speed/1000
        if pitch_to_next_point < math.radians(-4):
            throttle *= 0.95
        if pitch_to_next_point < math.radians(-6):
            throttle *= 0.95
        if pitch_to_next_point < math.radians(-8):
            throttle *= 0.93
        if pitch_to_next_point < math.radians(-10):
            throttle *= 0.92
        if pitch_to_next_point < math.radians(-11):
            throttle *= 0.92

        if pitch_to_next_point > math.radians(2):
            throttle *= 1.03
        if pitch_to_next_point < math.radians(3):
            throttle *= 1.03
        if pitch_to_next_point < math.radians(4):
            throttle *= 1.03
        if pitch_to_next_point < math.radians(5):
            throttle *= 1.03
        return throttle

    def _speed_for_turn(self, distance, target_speed, pitch_to_next_point):
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        d = (1/675) * (target_speed**2) + distance
        max_speed = math.sqrt(675 * d)
        return SpeedData(distance, current_speed, target_speed, max_speed)

    def _get_next_interesting_waypoints(self, more_waypoints: [Transform], current_speed):
        points = []
        dist = [] # for debugging
        start = self.agent.vehicle.transform
        points.append(start)
        curr_dist = 0
        prev_dist = 0
        num_points = 0
        for p in more_waypoints:
            end = p
            num_points += 1
            curr_dist += start.location.distance(end.location)
            if curr_dist > self.intended_target_distance[len(points)]:
            # this way was slower in first 2 sections.
            # if curr_dist > prev_dist + (self.intended_target_distance[len(points)] - self.intended_target_distance[len(points)-1]):
                self.target_distance[len(points)] = curr_dist
                prev_dist = curr_dist
                points.append(end)
                dist.append(curr_dist)
            start = end
            if len(points) >= len(self.target_distance):
                break

        # print("\n\nusing num_points= " + str(len(points)))
        self.dprint("wp dist " +  str(dist))
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

    def _get_target_speed(self, radius, pitch=0.0):
        if radius >= self.max_radius:
            return self.max_speed
        mu = 1.16
        target_speed = math.sqrt(mu*9.81*radius) * 3.6
        return max(20, min(target_speed, self.max_speed))  # clamp between 20 and max_speed

    def print_speed(self, text, s1, s2, s3, curr_s):
        self.dprint(text + " s1= " + str(round(s1, 2)) + " s2= " + str(round(s2, 2)) + " s3= " + str(round(s3, 2))
            + " cspeed= " + str(round(curr_s, 2)))

    # debug print
    def dprint(self, text):
        if PIDFastController.display_debug:
            PIDFastController.debug_strings.append(text)

    # static debug print, to store debug text from LatPIDController
    @staticmethod
    def sdprint(text):
        if PIDFastController.display_debug:
            PIDFastController.debug_strings.append(text)

    # brake test. print time, speed, distance after hitting brakes at some initial speed. 
    # def run_in_series_brake_test(self, 
    #                   next_waypoint: Transform, 
    #                   close_waypoint: Transform, 
    #                   far_waypoint: Transform, 
    #                   more_waypoints: [Transform], **kwargs) -> VehicleControl:

    #     # run lat pid controller
    #     steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        
    #     current_speed = Vehicle.get_speed(self.agent.vehicle)
    #     throttle = 1
    #     brake = 0
    #     if current_speed > 250 and self.brake_counter == 0:
    #         throttle = -1
    #         brake = 1
    #         self.brake_counter = 1
    #         self.brake_start = self.agent.vehicle.transform
    #     elif self.brake_counter > 0:
    #         throttle = -1
    #         brake = 1
    #         self.brake_counter += 1
        
    #     if current_speed > 1 and self.brake_counter > 0 and (self.brake_counter) % 5 == 1:
    #         break_dist = self.brake_start.location.distance(self.agent.vehicle.transform.location)
    #         print("Break test: " + str(self.brake_counter - 1) + " s= " + str(round(current_speed, 1)) + " d= " + str(round(break_dist, 1)))
    #     gear = 1
    #     return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)


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

        k_p, k_d, k_i = PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
        
        PIDFastController.sdprint("steer: " + str(lat_control) + " err" + str(error) + " k_p=" + str(k_p) + " de" + str(_de) + " k_d=" + str(k_d) 
            + " ie" + str(_ie) + " k_i=" + str(k_i) + " sum" + str(sum(self._error_buffer)))

        return lat_control, error, wide_error, sharp_error
