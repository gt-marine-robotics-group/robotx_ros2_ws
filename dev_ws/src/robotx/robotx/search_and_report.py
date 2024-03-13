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
# import ros_numpy as rnp
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
	# will return the gps coordinates for a point a given distance and bearing from start
	R = 6378.1
	lat1 = math.radians(start[0]) #Current lat point converted to radians
	lon1 = math.radians(start[1]) #Current long point converted to radians

	lat2 = math.asin( math.sin(lat1)*math.cos(distance/R) +
     math.cos(lat1)*math.sin(distance/R)*math.cos(bearing))

	lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(distance/R)*math.cos(lat1),
             math.cos(distance/R)-math.sin(lat1)*math.sin(lat2))

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
		###########################
		# vehicle connection
		print('Baby shark tutututututututututu')
		vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
		drone = Drone(vehicle)
		print("Ca c'est des vrais couteaux")
		print('Connected!')
		time.sleep(5.0)
		# print('cv2:' + cv2.__version__)

		####### add in the parameters ######
		# vehicle.parameters['PLND_ENABLED']=1
		# vehicle.parameters['PLND_TYPE']=4
		# # vehicle.parameters['PLND_EST_TYPE']=0
		# vehicle.parameters['LAND_SPEED']= 30 # cm/s 
		# vehicle.parameters['SIM_SONAR_SCALE'] = 10 
		# vehicle.parameters['RNGFND1_TYPE'] = 1 
		# vehicle.parameters['RNGFND_SCALING'] = 10 
		# vehicle.parameters['RNGFND1_PIN'] = 0 
		# vehicle.parameters['RNGFND1_MAX_CM'] = 300
		# vehicle.parameters['RNGFND1_MIN_CM'] = 0 

		vehicle.parameters['PLND_ENABLED']=1
		vehicle.parameters['PLND_TYPE']=1
		# vehicle.parameters['PLND_TYPE']=4
		vehicle.parameters['PLND_EST_TYPE']=0
		vehicle.parameters['LAND_SPEED']= 20 # cm/s 

		# set the rangefinder parameters 
		vehicle.parameters["RNGFND1_TYPE"] = 1 
		vehicle.parameters["RNGFND1_MIN_CM"] = 0 
		vehicle.parameters["RNGFND1_MAX_CM"] = 400 
		vehicle.parameters["RNGFND1_PIN"] = 0  
		vehicle.parameters["RNGFND1_SCALING"] = 12.12 
		vehicle.parameters['WPNAV_SPEED'] = 500.0  # cm/s

		##########################
		##########################

		velocity = 8.0
		takeoff_height = 5.0 #m 
		# vehicle.parameters["RNGFND1_MAX_CM"] = 0.8*takeoff_height
		#########################

		land_id_to_find = 72 
		marker_size = 20 # cm

		aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		parameters = aruco.DetectorParameters()


		horizontal_res = 640 
		vertical_res = 480 

		horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
		vertical_fov = 48.8 * (math.pi / 180)  # convert to radians 

		found_count = 0 
		notfound_count = 0 


		######### info on the markers to find #######
		marker1_id = "N"
		marker2_id = "R" 
		markers_id = [marker1_id, marker2_id]
		marker1_found = False 
		marker2_found = False 
		drone.search_now = True  # only look for text while the drone is stationary at the waypoint

		#############################################

		######### camera intrinsics #######

		dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
		camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
		np_camera_matrix = np.array(camera_matrix)
		np_dist_coeff = np.array(dist_coeff)

		time_last = 0 
		time_to_wait = 0.1 

	
		# record this location for the return 
		home_gps = drone.vehicle.location.global_relative_frame
		lat_home = drone.vehicle.location.global_relative_frame.lat
		lon_home = drone.vehicle.location.global_relative_frame.lon
		# print(home_gps.lat)
		wp_home = LocationGlobalRelative(lat_home, lon_home, takeoff_height)
		heading = drone.vehicle.heading  # degrees 

		test_bounds = {
            'north': 20,
            'south': -20,
            'east': -15,
            'west': 15
        }

		north_gps = get_gps_from_distance([lat_home, lon_home], 0, np.abs(test_bounds['north']) / 1000.0)  # North, bearing = 0 radians
		south_gps = get_gps_from_distance([lat_home, lon_home], math.pi, np.abs(test_bounds['south']) / 1000.0)  # South, bearing = π radians
		east_gps = get_gps_from_distance([lat_home, lon_home], math.pi / 2, np.abs(test_bounds['east']) / 1000.0)  # East, bearing = π/2 radians
		west_gps = get_gps_from_distance([lat_home, lon_home], 3 * math.pi / 2, np.abs(test_bounds['west']) / 1000.0)  # West, bearing = 3π/2 radians

		# Define the test_bounds in GPS coordinates
		test_bounds_gps = {
			'north': north_gps[0],
			'south': south_gps[0],
			'east': east_gps[1],
			'west': west_gps[1]
		}

		grid_waypoints = drone.generate_grid_search_waypoints(test_bounds_gps)

		# download current list of commands from the drone were currently connected to 
		cmds = vehicle.commands
		cmds.download()
		cmds.wait_ready()

		# clear the current list of commands 
		cmds.clear()
		print("Hello le drone, on va chercher")
		for waypoint in grid_waypoints:
			lon = waypoint[1]
			lat = waypoint[0]
						# lon, lat = raster.xy(i, col[j])
			cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,0,0,0,0,0,lat,lon,takeoff_height)
			cmds.add(cmd)

		cmd4 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,0,0,0,0,0,0,0,0)
		# cmd4 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,0,0,0,0,0,wphome.lat,wphome.lon,wphome.alt)

		end_lat, end_lon = grid_waypoints[-1][0], grid_waypoints[-1][1]


		cmds.add(cmd4)

		# upload the commands 
		drone.vehicle.commands.upload()

		# takeoff 
		print("Taking off...")
		drone.arm_and_takeoff()
		time.sleep(3.0)


		# execture the mission

		# save the home location, for some reason ardupilot is stupid af so its in here twice
		wphome = drone.vehicle.location.global_relative_frame
		start_lat, start_lon = wphome.lat, wphome.lon 


		print("Switching to auto mode...")
		drone.vehicle.mode = VehicleMode("AUTO")
		while drone.vehicle.mode != "AUTO":
			time.sleep(0.2)

		current_distance_from_end = 1000
		time.sleep(3)
		print("beginning search...")


		while current_distance_from_end > 1.0: 
			drone.search()
			rclpy.spin_once(drone)

			# search(vehicle)
			# check the location 
			current_location = vehicle.location.global_relative_frame
			current_distance_from_end = get_distance_meters(LocationGlobalRelative(end_lat, end_lon, takeoff_height), current_location)

		# sub.unregister()
		print('End of search exactly now')
		search_now = False

		# return to launch and land 
		print('returning to launch...')
		# dkf.goto(vehicle, home_gps)
		# dkf.return_to_launch(vehicle, home)

		# land 
		# print('Landing...')
		# precision_land(vehicle)
		drone.precision_land_landing_pad()

		print('Marker position determination.....')

		# determine the position of the markers from the recorded areas of interest 
		first_marker = np.asarray(drone.tag_positions[0])
		first_marker_identified = 0.0
		second_marker = np.array([0.0,0.0])
		secon_marker_identified = 0.0 
		for marker_id, point in drone.tag_positions.items():
			point_list = point.tolist()  # Convert numpy array to list if needed for compatibility
			distance_from_home = get_distance_meters(LocationGlobalRelative(point_list[0], point_list[1], 0), LocationGlobalRelative(wphome.lat, wphome.lon, 0))

			if distance_from_home < 1:
				continue

			distance_from_first_marker = get_distance_meters(LocationGlobalRelative(point_list[0], point_list[1], 0), LocationGlobalRelative(first_marker[0],first_marker[1], 0))
			if distance_from_first_marker < 2:
				first_marker[0] += point_list[0]
				first_marker[1] += point_list[1]
				first_marker_identified += 1.0 
			else:
				second_marker[0] += point_list[0]
				second_marker[1] += point_list[1]
				secon_marker_identified += 1.0 

		# Correct the average position calculation
		if first_marker_identified > 0:  # Ensure division by zero is avoided
			first_marker[0] /= first_marker_identified
			first_marker[1] /= first_marker_identified

		if secon_marker_identified > 0:  # Ensure division by zero is avoided
			second_marker[0] /= secon_marker_identified
			second_marker[1] /= secon_marker_identified

		print(first_marker)
		print(second_marker)


	except KeyboardInterrupt:
		print('Simulation interupted')
		pass

	finally:
		drone.destroy_node()  
		rclpy.shutdown()


if __name__ == "__main__":
	main()