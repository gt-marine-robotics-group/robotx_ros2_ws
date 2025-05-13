#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from array import array
import os
import pytesseract
from pytesseract import Output
from imutils.object_detection import non_max_suppression
from cv_bridge import CvBridge
import imutils
from robotx import drone_node
from robotx.drone_node import Drone

def get_gps_from_distance(start, bearing, distance):
    R = 6378.1
    lat1 = math.radians(start[0])
    lon1 = math.radians(start[1])

    lat2 = math.asin(math.sin(lat1)*math.cos(distance/R) + math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1), math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return lat2, lon2

def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5

def main():
    try:
        rclpy.init()
        print('Vehicle connection setup...')
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        drone = Drone(vehicle)
        print("Ca c'est des vrais couteaux")
        print('Connected!')
        time.sleep(5.0)

        vehicle.parameters['PLND_ENABLED'] = 1
        vehicle.parameters['PLND_TYPE'] = 1
        vehicle.parameters['PLND_EST_TYPE'] = 0
        vehicle.parameters['LAND_SPEED'] = 20  # cm/s

        vehicle.parameters["RNGFND1_TYPE"] = 1
        vehicle.parameters["RNGFND1_MIN_CM"] = 0
        vehicle.parameters["RNGFND1_MAX_CM"] = 400
        vehicle.parameters["RNGFND1_PIN"] = 0
        vehicle.parameters["RNGFND1_SCALING"] = 12.12
        vehicle.parameters['WPNAV_SPEED'] = 500.0  # cm/s

        velocity = 8.0
        takeoff_height = 5.0  # m

        land_id_to_find = 72
        marker_size = 20  # cm

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()

        horizontal_res = 640
        vertical_res = 480

        horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians
        vertical_fov = 48.8 * (math.pi / 180)  # convert to radians

        found_count = 0
        notfound_count = 0

        marker1_id = "N"
        marker2_id = "R"
        markers_id = [marker1_id, marker2_id]
        marker1_found = False
        marker2_found = False
        drone.search_now = True  # only look for text while the drone is stationary at the waypoint

        dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]]
        np_camera_matrix = np.array(camera_matrix)
        np_dist_coeff = np.array(dist_coeff)

        time_last = 0
        time_to_wait = 0.1

        home_gps = drone.vehicle.location.global_relative_frame
        lat_home = home_gps.lat
        lon_home = home_gps.lon

        wp_home = LocationGlobalRelative(lat_home, lon_home, takeoff_height)
        heading = drone.vehicle.heading  # degrees

        test_bounds = {
			'north': 20,
			'south': -20,
			'east': -15,
			'west': 15
		}

        north_gps = get_gps_from_distance([lat_home, lon_home], 0, np.abs(test_bounds['north']) / 1000.0)
        south_gps = get_gps_from_distance([lat_home, lon_home], math.pi, np.abs(test_bounds['south']) / 1000.0)
        east_gps = get_gps_from_distance([lat_home, lon_home], math.pi / 2, np.abs(test_bounds['east']) / 1000.0)
        west_gps = get_gps_from_distance([lat_home, lon_home], 3 * math.pi / 2, np.abs(test_bounds['west']) / 1000.0)

        test_bounds_gps = {
            'north': north_gps[0],
            'south': south_gps[0],
            'east': east_gps[1],
            'west': west_gps[1]
        }

        grid_waypoints = drone.generate_grid_search_waypoints(test_bounds_gps)

        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

        cmds.clear()
        print("Configuring waypoints for grid search...")
        for waypoint in grid_waypoints:
            lat, lon = waypoint[0], waypoint[1]
            cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, takeoff_height)
            cmds.add(cmd)

        cmd_return = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        cmds.add(cmd_return)

        vehicle.commands.upload()

        print("Taking off...")
        drone.arm_and_takeoff()

        print("Starting grid search...")
        vehicle.mode = VehicleMode("AUTO")
        while vehicle.mode != "AUTO":
            time.sleep(0.2)

        print("Searching...")
        end_lat, end_lon = grid_waypoints[-1][0], grid_waypoints[-1][1]
        current_distance_from_end = get_distance_meters(LocationGlobalRelative(end_lat, end_lon, takeoff_height), vehicle.location.global_relative_frame)
        while current_distance_from_end > 1.0 or drone.landing_allowed:
            drone.search()
            rclpy.spin_once(drone)
            current_location = vehicle.location.global_relative_frame
            current_distance_from_end = get_distance_meters(LocationGlobalRelative(end_lat, end_lon, takeoff_height), current_location)
            if drone.targeted_aruco_marker in drone.currently_seen_markers:
                drone.search_now = False
                drone.landing_allowed = True
                current_distance_from_end = 0.0

    except KeyboardInterrupt:
        print("Mission interrupted by user.")

    finally:
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()