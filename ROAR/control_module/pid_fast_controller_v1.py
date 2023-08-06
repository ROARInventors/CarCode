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

class SpeedUpdate(Enum):
    BIG_INCREASE = 5
    SMALL_INCREASE = 4 
    MAINTAIN = 3
    NO_UPDATE = 2
    SMALL_DECREASE = 1
    BIG_DECREASE = 0

class SpeedData:
    def __init__(self, distance_to_section, current_speed, target_speed, recommended_speed):
        super().__init__()
        self.current_speed = current_speed
        self.distance_to_section = distance_to_section
        self.target_speed_at_distance = target_speed
        self.recommended_speed_now = recommended_speed
        self.speed_diff = current_speed - recommended_speed
        self.speed_update = SpeedUpdate.NO_UPDATE

    def get_recommended_speed(self):
        return self.recommended_speed_now


class PIDFastController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_radius = 10000
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        # self.target_distance = [0, 50, 100, 150, 200, 250]  # 1 slower than 2
        self.target_distance = [0, 30, 60, 90, 120, 150]  # 2 good 
        # self.target_distance = [0, 20, 40, 70, 100, 150]  # 3 slower than 2
        # self.target_distance = [0, 20, 40, 60, 80, 100]  # 4 slower than 2
        self.close_index = 0
        self.mid_index = 1
        self.far_index = 2
        self.tick_counter = 0
        self.previous_throttle_brake = (1, 0)
        self.previous_speed = 0
        self.brake_counter = 0

        self.region = 1
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
            dt=0.05, # not sure if this is good?
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    # modified
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

        self.tick_counter += 1
        if self.region == 1:
            throttle, brake = self._get_throttle_and_brake(more_waypoints, wide_error)
        elif self.region == 2:
            waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                self.brake_counter += 1
                if self.brake_counter >= 3:
                    self.brake_counter = 0
            elif dist <= 5:
                self.brake_counter = 1
                throttle = -1
                brake = 1
                print("\nspecial break point: ")
                print(self.waypoint_queue_braking[0])
                self.waypoint_queue_braking.pop(0)
            else:
                throttle, brake = self._get_throttle_and_brake(more_waypoints, wide_error)
        
        # gear = 1        
        gear = max(1, (int)((current_speed - 2*pitch) / 60))
        if throttle == -1:
            gear = -1
        
        waypoint = self.waypoint_queue_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist <= 10:
            self.region += 1
            self.waypoint_queue_region.pop(0)

        #throttle, brake = self._brake_test(throttle, brake)
        # if keyboard.is_pressed("space"):
        #      print(self.agent.vehicle.transform.record())
        self.previous_throttle_brake = (throttle, brake)
        self.previous_speed = current_speed
        if self.brake_counter > 0 and brake > 1:
            self.brake_counter -= 1

        self.dprint("--- " + str(throttle) + " " + str(brake)) 

        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)

    def _get_throttle_and_brake(self, more_waypoints: [Transform], wide_error):
        # return self._get_throttle_and_brake_based_on_distance(more_waypoints, wide_error)
        return self._get_throttle_and_brake_based_on_speed(more_waypoints, wide_error)

    def _get_throttle_and_brake_based_on_speed(self, more_waypoints: [Transform], wide_error):
        current_speed = Vehicle.get_speed(self.agent.vehicle)

        wp = self._get_next_interesting_waypoints(more_waypoints, current_speed)
        r1 = self._get_radius(wp[self.close_index : self.close_index + 3])
        r2 = self._get_radius(wp[self.mid_index : self.mid_index + 3])
        r3 = self._get_radius(wp[self.far_index : self.far_index + 3])

        pitch_to_next_point = \
            math.asin( (wp[2].location.y - wp[0].location.y) / wp[0].location.distance(wp[2].location))

        target_speed1 = self._get_target_speed(r1, pitch_to_next_point)
        target_speed2 = self._get_target_speed(r2, pitch_to_next_point)
        target_speed3 = self._get_target_speed(r3, pitch_to_next_point)

        close_distance = self.target_distance[self.close_index] + 5
        mid_distance = self.target_distance[self.mid_index]
        far_distance = self.target_distance[self.far_index]
        speed_data = []
        speed_data.append(self._speed_for_turn(close_distance, target_speed1, pitch_to_next_point))
        speed_data.append(self._speed_for_turn(mid_distance, target_speed2, pitch_to_next_point))
        speed_data.append(self._speed_for_turn(far_distance, target_speed3, pitch_to_next_point))

        update = self._select_data(speed_data)
        self.dprint("r1=" + str(round(r1)) + " r2=" + str(round(r2)) + " r3= " + str(round(r3)) + " ts1= " 
              + str(round(target_speed1, 2)) + " ts2= " + str(round(target_speed2, 2)) + " ts3= " + str(round(target_speed3, 2))
              + " updates " + str(speed_data) + " upd " + str(update))

        t, b = self._speed_data_to_throttle_and_brake(update, pitch_to_next_point)

    def _speed_data_to_throttle_and_brake(self, speed_data: SpeedData, pitch_to_next_point):
        percent_of_max = speed_data.current_speed / speed_data.recommended_speed_now

        self.dprint("dist=" + str(round(speed_data.distance_to_section)) + " cs=" + str(round(speed_data.current_speed, 2)) 
                    + " ts= " + str(round(speed_data.target_speed_at_distance, 2)) 
                    + " maxs= " + str(round(speed_data.recommended_speed_now, 2)) + " pcnt= " + str(round(percent_of_max, 2)))

        # TODO: adjust for pitch
        percent_change_per_tick = 0.06 # speed drop for one time tick of braking
        if percent_of_max > 1:
            # Consider slowing down
            if percent_of_max > 1 + percent_change_per_tick:
                if self.brake_counter == 0:
                    self.brake_counter = math.floor((percent_of_max - 1) / 0.06)
                    return -1, 1
                elif self.brake_counter * percent_change_per_tick:
                    
                    


            if self.brake_counter == 0:
              # start braking
            if self.brake_counter > 0:
                # already braking

        else:
            # Consider speeding up

        if 0.98 < percent_of_max and percent_of_max < 1.02:
            return SpeedUpdate.MAINTAIN
        if percent_of_max < 1.0:
            if percent_of_max < 0.90:
                return SpeedUpdate.BIG_INCREASE
            else:
                return SpeedUpdate.SMALL_INCREASE
        if percent_of_max > 1.0:
            if percent_of_max > 1.15:
                return SpeedUpdate.BIG_DECREASE
            else:
                return SpeedUpdate.SMALL_DECREASE

        # return SpeedUpdate.NO_UPDATE
        return 1, 0

    def _get_throttle_and_brake_based_on_speed_old(self, more_waypoints: [Transform], wide_error):
        current_speed = Vehicle.get_speed(self.agent.vehicle)

        wp = self._get_next_interesting_waypoints(more_waypoints, current_speed)
        r1 = self._get_radius(wp[self.close_index : self.close_index + 3])
        r2 = self._get_radius(wp[self.mid_index : self.mid_index + 3])
        r3 = self._get_radius(wp[self.far_index : self.far_index + 3])

        pitch_to_next_point = \
            math.asin( (wp[2].location.y - wp[0].location.y) / wp[0].location.distance(wp[2].location))

        target_speed1 = self._get_target_speed(r1, pitch_to_next_point)
        target_speed2 = self._get_target_speed(r2, pitch_to_next_point)
        target_speed3 = self._get_target_speed(r3, pitch_to_next_point)

        close_distance = self.target_distance[self.close_index] + 5
        mid_distance = self.target_distance[self.mid_index]
        far_distance = self.target_distance[self.far_index]
        speed_updates = []
        speed_updates.append(self._speed_update_for_turn(close_distance, target_speed1, pitch_to_next_point))
        speed_updates.append(self._speed_update_for_turn(mid_distance, target_speed2, pitch_to_next_point))
        speed_updates.append(self._speed_update_for_turn(far_distance, target_speed3, pitch_to_next_point))

        update = self._select_update(speed_updates)
        self.dprint("r1=" + str(round(r1)) + " r2=" + str(round(r2)) + " r3= " + str(round(r3)) + " ts1= " 
              + str(round(target_speed1, 2)) + " ts2= " + str(round(target_speed2, 2)) + " ts3= " + str(round(target_speed3, 2))
              + " updates " + str(speed_updates) + " upd " + str(update))

        t, b = self._speed_update_to_throttle_and_brake(update, current_speed, pitch_to_next_point)


        # t, b = self._get_throttle_updates(current_speed, target_speed1, target_speed2, target_speed3, pitch_to_next_point, wide_error)
        # if t < 1.0:
        #     pitch = self.agent.vehicle.transform.rotation.pitch
        #     print("\npitch= " + str(pitch))
        #     print("\n\ngot radius " + str(r1) + " " + str(r2) 
        #         + " tspeed1= " + str(target_speed1) 
        #         + " tspeed2= " + str(target_speed2) 
        #         + " tspeed3= " + str(target_speed3)
        #         + " cspeed= " +  (str(current_speed)) 
        #         + " t= " + str(t) + " b= " + str(b)
        #         #   + " wp= " + str(wp)
        #         )
        return t, b

    def _select_speed(self, speed_data: SpeedData):
        # return speed data with the largest speed excess (i.e. current > recommended)
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

    def _select_update(self, speed_updates):
        cautious = self._get_most_cautious(speed_updates)
        if cautious == SpeedUpdate.BIG_DECREASE or cautious == SpeedUpdate.SMALL_DECREASE:
            return cautious
        # if close distance looks ok, and longer distance is clear start throttle early
        # hope to come out of the turns faster
        # TODO: may need to count that this condition is true for N time ticks before hitting throttle 
        if speed_updates[2] == SpeedUpdate.BIG_INCREASE:
            return SpeedUpdate.BIG_INCREASE

    def _get_most_cautious(self, speed_updates):
        # check them in this order.
        for update in [SpeedUpdate.BIG_DECREASE, 
                       SpeedUpdate.SMALL_DECREASE, 
                       SpeedUpdate.NO_UPDATE, 
                       SpeedUpdate.MAINTAIN, 
                       SpeedUpdate.SMALL_INCREASE, 
                       SpeedUpdate.BIG_INCREASE]:
            if update in speed_updates:
                return update
        # just in case...
        return SpeedUpdate.NO_UPDATE
    
    def _speed_update_to_throttle_and_brake(self, update, current_speed, pitch_to_next_point):
        if update == SpeedUpdate.NO_UPDATE:
            return self.previous_throttle_brake
        if update == SpeedUpdate.BIG_DECREASE:
            return -1, 1
        if update == SpeedUpdate.SMALL_DECREASE:
            # return -1, 1
            if current_speed > 100:
                return 1, 0.5
            else:
                return 0, 0
        if update == SpeedUpdate.BIG_INCREASE:
            return 1, 0
        maintain_throttle = self._get_throttle_to_maintain_speed(current_speed, pitch_to_next_point)
        if update == SpeedUpdate.MAINTAIN:
            return maintain_throttle, 0
        if update == SpeedUpdate.SMALL_INCREASE:
            return maintain_throttle * 1.05, 0
        return 1, 0

    def _get_throttle_to_maintain_speed(self, current_speed, pitch_to_next_point):
        # TODO: commpute throttle needed to maintain current speed with given pitch.
        #       need to consider current_speed
        throttle = 1
        if pitch_to_next_point > math.radians(2):
            throttle = 0.8 # on downhill, reduce default throttle
        if pitch_to_next_point > math.radians(2.5):
            throttle = 0.7  # on downhill, reduce default throttle
        if pitch_to_next_point > math.radians(3.5):
            throttle = 0.6  # on downhill, reduce default throttle
        return throttle

    def _speed_for_turn(self, distance, target_speed, pitch_to_next_point):
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        d = (1/675) * (target_speed**2) + distance
        # TODO: figure out how to account for pitch
        # pitch_multiplier = 1 + (0.02 * round(math.degrees(pitch_to_next_point)))
        # d = (1/675) * (target_speed**2) + distance*pitch_multiplier
        max_speed = math.sqrt(675 * d)
        return SpeedData(distance, current_speed, target_speed, max_speed)

    def _speed_update_for_turn(self, distance, target_speed, pitch_to_next_point):
        # delta_percent = (target_speed - current_speed)/target_speed
        # if abs(delta_percent) < 0.005:
        #     return SpeedUpdate.NO_UPDATE
        # if delta_percent < 0.02:
        #     return SpeedUpdate.MAINTAIN
        
        # cfrom250 = (-1/675) * (current_speed**2) + 95 #distance from 250 to current speed
        # tfrom250 = (-1/675) * (target_speed**2) + 95 #distance from 250 to target speed
        # required_distance = tfrom250 - cfrom250 #distance from current speed to target

        current_speed = Vehicle.get_speed(self.agent.vehicle)
        d = (1/675) * (target_speed**2) + distance
        # TODO: figure out how to account for pitch
        # pitch_multiplier = 1 + (0.02 * round(math.degrees(pitch_to_next_point)))
        # d = (1/675) * (target_speed**2) + distance*pitch_multiplier
        max_speed = math.sqrt(675 * d)

        percent_of_max = current_speed / max_speed

        self.dprint("dist=" + str(round(distance)) + " cs=" + str(round(current_speed, 2)) 
                    + " ts= " + str(round(target_speed, 2)) 
                    + " maxs= " + str(round(max_speed, 2)) + " pcnt= " + str(round(percent_of_max, 2)))

        if 0.98 < percent_of_max and percent_of_max < 1.02:
            return SpeedUpdate.MAINTAIN
        if percent_of_max < 1.0:
            if percent_of_max < 0.90:
                return SpeedUpdate.BIG_INCREASE
            else:
                return SpeedUpdate.SMALL_INCREASE
        if percent_of_max > 1.0:
            if percent_of_max > 1.15:
                return SpeedUpdate.BIG_DECREASE
            else:
                return SpeedUpdate.SMALL_DECREASE

        return SpeedUpdate.NO_UPDATE


    def _get_throttle_and_brake_based_on_distance(self, more_waypoints: [Transform], wide_error):
        current_speed = Vehicle.get_speed(self.agent.vehicle)

        wp = self._get_next_interesting_waypoints(more_waypoints, current_speed)
        r1 = self._get_radius(wp[0:3])
        r2 = self._get_radius(wp[1:4])
        r3 = self._get_radius(wp[2:5])
        target_speed1 = self._get_target_speed(r1)
        target_speed2 = self._get_target_speed(r2)
        target_speed3 = self._get_target_speed(r3)
        self.dprint("r1=" + str(round(r1)) + " r2=" + str(round(r2)) + " r3= " + str(round(r3)) + " ts1= " 
              + str(round(target_speed1, 2)) + " ts2= " + str(round(target_speed2, 2)) + " ts3= " + str(round(target_speed3, 2)))

        pitch_to_next_point = \
            math.asin( (wp[2].location.y - wp[0].location.y) / wp[0].location.distance(wp[2].location))
            # math.acos( (wp[2].location.y - wp[0].location.y) / wp[0].location.distance(wp[2].location))
        t, b = self._get_throttle_updates(current_speed, target_speed1, target_speed2, target_speed3, pitch_to_next_point, wide_error)
        # if t < 1.0:
        #     pitch = self.agent.vehicle.transform.rotation.pitch
        #     print("\npitch= " + str(pitch))
        #     print("\n\ngot radius " + str(r1) + " " + str(r2) 
        #         + " tspeed1= " + str(target_speed1) 
        #         + " tspeed2= " + str(target_speed2) 
        #         + " tspeed3= " + str(target_speed3)
        #         + " cspeed= " +  (str(current_speed)) 
        #         + " t= " + str(t) + " b= " + str(b)
        #         #   + " wp= " + str(wp)
        #         )
        return t, b
    
    def _get_next_interesting_waypoints(self, more_waypoints: [Transform], current_speed):
        # target_distance = [0, 30, 60, 110, 160, 210, 260]
        points = []
        start = self.agent.vehicle.transform
        points.append(start)
        curr_dist = 0
        num_points = 0
        for p in more_waypoints:
            end = p
            num_points += 1
            curr_dist += start.location.distance(end.location)
            if curr_dist > self.target_distance[len(points)]:
                points.append(end)
            start = end
            if len(points) >= len(self.target_distance):
                break

        # print("\n\nusing num_points= " + str(len(points)))
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
        
    def get_dip_radius(self, wp: [Transform]):
        point1 = (wp[0].location.y, 0)
        point2 = (wp[1].location.y, 30)
        point3 = (wp[2].location.y, 60)

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
        if point2[0] < (point1[0]+point2[0])/2:
            return -radius
        return radius
    
    def get_flight_time(self, current_speed, angle = 5):
        #assume 5 degree angle
        current_speed = current_speed / 3.6
        vertical_speed = math.sin(math.radians(angle)) * current_speed
        time = vertical_speed/9.81*2
        return time

    def _get_target_speed(self, radius, pitch=0.0):
        if radius >= self.max_radius:
            return self.max_speed
        mu = 0.8
        # mu = 0.75  # try this
        target_speed = math.sqrt(mu*9.81*radius) * 3.6
        if abs(pitch) > math.radians(1.5):
            # TODO: fix me!
            multiplier = max(0.8, min((1 + 0.5*math.tan(pitch)), 1.2))
            # print("pitch " + str(pitch) + " ap " + str(abs(pitch)) + " pdeg " + str(math.degrees(pitch)) + " tan " + str(math.tan(pitch)) + " mult " + str(multiplier))
            # target_speed *= multiplier
        return max(5, min(target_speed, self.max_speed))  # clamp between 5 and max_speed

    # only brake
    def _get_throttle_updates(self, current_speed, target_speed1, target_speed2, target_speed3, pitch_to_next_point, wide_error):
        throttle = 1
        brake = 0
        if pitch_to_next_point > math.radians(2):
            throttle = 0.9 # on downhill, reduce default throttle
        if pitch_to_next_point > math.radians(2.5):
            throttle = 0.8  # on downhill, reduce default throttle
        if pitch_to_next_point > math.radians(3.5):
            throttle = 0.7  # on downhill, reduce default throttle
        if (current_speed + 2 < target_speed1):
            throttle = 1
            brake = 0
            self.print_speed("throttle1", target_speed1, target_speed2, target_speed3, current_speed)
        if (current_speed < target_speed1) and (current_speed < target_speed2): #and (current_speed < target_speed3):
            throttle = 1
            brake = 0
            self.print_speed("throttle2", target_speed1, target_speed2, target_speed3, current_speed)
        if self._need_to_brake(self.target_distance[0] + 5, current_speed, target_speed1, pitch_to_next_point):
            throttle = -1
            brake = 1
            self.print_speed("break1", target_speed1, target_speed2, target_speed3, current_speed)
        if self._need_to_brake(self.target_distance[1] + 15, current_speed, target_speed2, pitch_to_next_point):
            throttle = -1
            brake = 1
            self.print_speed("break2", target_speed1, target_speed2, target_speed3, current_speed)
        if self._need_to_brake(self.target_distance[2] + 15, current_speed, target_speed3, pitch_to_next_point):
           throttle = -1
           brake = 1
           self.print_speed("break3", target_speed1, target_speed2, target_speed3, current_speed)
        self.dprint("---  " + str(throttle) + "  " + str(brake))
        return throttle, brake

    def print_speed(self, text, s1, s2, s3, curr_s):
        self.dprint(text + " s1= " + str(round(s1, 2)) + " s2= " + str(round(s2, 2)) + " s3= " + str(round(s3, 2))
            + " cspeed= " + str(round(curr_s, 2)))
        # if self.tick_counter > 200:
        #     print(text + " s1= " + str(round(s1, 2)) + " s2= " + str(round(s2, 2)) + " s3= " + str(round(s3, 2))
        #         + " cspeed= " + str(round(curr_s, 2)))

    # debug print
    def dprint(self, text):
        if self.tick_counter < 0:
        # if self.tick_counter > 400:
            print(text)

    def _need_to_brake(self, distance_to_section, current_speed, target_speed, pitch_to_next_point):
        delta = current_speed - target_speed
        if delta < 2:
            return False

        #equation for distance to brake from 250: (-1/675)x^2+95
        cfrom250 = (-1/675) * (current_speed**2) + 95 #distance from 250 to current speed
        tfrom250 = (-1/675) * (target_speed**2) + 95 #distance from 250 to target speed

        required_distance = tfrom250 - cfrom250 #distance from current speed to target
        if distance_to_section > 70:
            self.dprint(" d= " + str(distance_to_section) + " c= " + str(round(cfrom250, 2)) + " t= " + str(round(tfrom250, 2)) 
                  + " req= " + str(round(required_distance, 2)))
        return distance_to_section < required_distance



    # original
    def run_in_series_old(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> VehicleControl:

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
            if sharp_error < 0.68 or current_speed <= 90:
                throttle = 1
                brake = 0
            else:
                throttle = -1
                brake = 1
        elif self.region == 2:
            waypoint = self.waypoint_queue_braking[0] # 5012 is weird bump spot
            dist = self.agent.vehicle.transform.location.distance(waypoint.location)
            if dist <= 5:
                self.brake_counter = 1
                # print(self.waypoint_queue_braking[0])
                self.waypoint_queue_braking.pop(0)
            if self.brake_counter > 0:
                throttle = -1
                brake = 1
                self.brake_counter += 1
                if self.brake_counter >= 4:
                    self.brake_counter = 0
            elif sharp_error >= 0.67 and current_speed > 70:
                throttle = 0
                brake = 0.4
            elif wide_error > 0.09 and current_speed > 92: # wide turn
                throttle = max(0, 1 - 6*pow(wide_error + current_speed*0.003, 6))
                brake = 0
            else:
                throttle = 1
                brake = 0
        
        gear = max(1, (int)((current_speed - 2*pitch) / 60))
        if throttle == -1:
            gear = -1
        
        waypoint = self.waypoint_queue_region[0]
        dist = self.agent.vehicle.transform.location.distance(waypoint.location)
        if dist <= 10:
            self.region += 1
            self.waypoint_queue_region.pop(0)
        
        # if keyboard.is_pressed("space"):
        #      print(self.agent.vehicle.transform.record())
        
        return VehicleControl(throttle=throttle, steering=steering, brake=brake, gear=gear)

    def _brake_test(self, original_throttle, original_brake):
        throttle, brake = original_throttle, original_brake
        
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        if self.brake_test_in_progress is False:
            if current_speed > 150:  # when to start the test
                self.brake_test_in_progress = True
                self.brake_counter = 1
                self.brake_start = self.agent.vehicle.transform
                throttle, brake = -1, 1
                print("-------")
        else:
            if self.brake_counter < 4:  # how many time ticks to use brake
                throttle, brake = -1, 1
            if self.brake_counter >= 25:  # for how many time tick to print output
                self.brake_test_in_progress = False
            self.brake_counter += 1
        
        if self.brake_test_in_progress is True:
        # if self.brake_test_in_progress is True and (self.brake_counter) % 5 == 1:
            break_dist = self.brake_start.location.distance(self.agent.vehicle.transform.location)
            x = self.agent.vehicle.transform.location.x
            #print("Brake test: " + str(self.brake_counter - 1) + " x= " + str(x) + " thr= " + str(throttle)
            #     + " s= " + str(round(current_speed, 1)) + " d= " + str(round(break_dist, 1)))
            print(str(self.brake_counter) + ',' + str(current_speed))

        return throttle, brake

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
        # wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

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
        # sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

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
        return lat_control, error, wide_error, sharp_error
    




    #  An idea to try to coast sometimes
    #
    # def _get_throttle_updates_coast(self, current_speed, target_speed1, target_speed2, target_speed3, pitch_to_next_point, wide_error):
    #     throttle = 1
    #     brake = 0
    #     if pitch_to_next_point > math.radians(2.5):
    #         throttle = 0.8
    #     if pitch_to_next_point > math.radians(2):
    #         throttle = 0.9
    #     if (current_speed + 2 < target_speed1):
    #         throttle = 1
    #         brake = 0
    #         self.print_speed("throttle1", target_speed1, target_speed2, target_speed3, current_speed)
    #     if (current_speed < target_speed1) and (current_speed < target_speed2) and (current_speed < target_speed3):
    #         throttle = 1
    #         brake = 0
    #         self.print_speed("throttle2", target_speed1, target_speed2, target_speed3, current_speed)
    #     recommendations = [
    #         self._throttle_change(self.target_distance[0] + 5, current_speed, target_speed1, pitch_to_next_point),
    #         self._throttle_change(self.target_distance[1], current_speed, target_speed2, pitch_to_next_point),
    #         self._throttle_change(self.target_distance[3], current_speed, target_speed3, pitch_to_next_point)
    #     ]
    #     if "break" in recommendations:
    #         throttle = -1
    #         brake = 1
    #         self.print_speed(
    #             "break-" + str(recommendations.index("break")), target_speed1, target_speed2, target_speed3, current_speed)
    #     elif "coast" in recommendations:
    #         throttle = 0
    #         brake = 0
    #         self.print_speed(
    #             "coast-" + str(recommendations.index("coast")), target_speed1, target_speed2, target_speed3, current_speed)
    #     elif "gentle_throttle" in recommendations:
    #         throttle = 0.5
    #         brake = 0
    #         self.print_speed(
    #             "gentle-" + str(recommendations.index("gentle_throttle")), target_speed1, target_speed2, target_speed3, current_speed)

    #     self.dprint("---  " + str(throttle) + "  " + str(brake))
    #     return throttle, brake


    # def _throttle_change(self, distance_to_section, current_speed, target_speed, pitch_to_next_point):
    #     delta = current_speed - target_speed
    #     if delta < 1:
    #         return "throttle"

    #     #equation for distance to brake from 250: (-1/675)x^2+95
    #     cfrom250 = (-1/675) * (current_speed**2) + 95 #distance from 250 to current speed
    #     tfrom250 = (-1/675) * (target_speed**2) + 95 #distance from 250 to target speed

    #     required_distance = tfrom250 - cfrom250 #distance from current speed to target
    #     if distance_to_section > 0:
    #         self.dprint(" d= " + str(distance_to_section) + " c= " + str(round(cfrom250, 2)) + " t= " + str(round(tfrom250, 2)) 
    #               + " req= " + str(round(required_distance, 2)))

    #     ok_to_coast_delta = 0.02 * distance_to_section
    #     if required_distance - 1 < distance_to_section:
    #         return "throttle"
    #     if required_distance < distance_to_section:
    #         return "gentle_throttle"
    #     if required_distance < distance_to_section + ok_to_coast_delta:
    #         return "coast"
    #     else:
    #         return "break"
