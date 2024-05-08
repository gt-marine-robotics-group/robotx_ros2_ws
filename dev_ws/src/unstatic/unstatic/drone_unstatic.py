#!/usr/bin/env python3

# import exceptions
import argparse 
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image 
import cv2
import cv2.aruco as aruco
import apriltag
import sys
import time
import math
import numpy as np
import ros2_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil	
from array import array 
from cv_bridge import CvBridge
import imutils
from imutils.object_detection import non_max_suppression
import pytesseract
from rclpy.qos import QoSProfile
from std_msgs.msg import String 
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
import json
import os
from geometry_msgs.msg import PoseStamped, PoseArray, Point
from rclpy.executors import MultiThreadedExecutor
# from robotx.msg import MissionState
# from robotx.action import DroneOperation  # Replace with your actual package name and action file name




def get_distance_meters(targetLocation, currentLocation):
			dLat = targetLocation.lat - currentLocation.lat
			dLon = targetLocation.lon - currentLocation.lon

			return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5


def thin_font(image):
	image = cv2.bitwise_not (image)
	kernel = np.ones((2,2), np.uint8)
	image = cv2.erode(image, kernel, iterations=3)
	image = cv2.bitwise_not(image)
	return(image)

def noise_removal(image):
	kernel = np.ones((1,1), np.uint8)
	image = cv2.dilate(image, kernel, iterations=1)
	kernel = np.ones((1,1), np.uint8)
	image = cv2.erode(image, kernel, iterations=1)
	image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
	image = cv2.medianBlur(image, 3)
	return image

def get_grayscale(image):
	return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

def thresholding(image):
	return cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]


def decode_predictions(scores, geometry):
	# grab the number of rows and columns from the scores volume, then
	# initialize our set of bounding box rectangles and corresponding
	# confidence scores
	(numRows, numCols) = scores.shape[2:4]
	rects = []
	confidences = []
	# loop over the number of rows
	for y in range(0, numRows):
		# extract the scores (probabilities), followed by the
		# geometrical data used to derive decode_predictionspotential bounding box
		# coordinates that surround text
		scoresData = scores[0, 0, y]
		xData0 = geometry[0, 0, y]
		xData1 = geometry[0, 1, y]
		xData2 = geometry[0, 2, y]
		xData3 = geometry[0, 3, y]
		anglesData = geometry[0, 4, y]
		# loop over the number of columns
		for x in range(0, numCols):
			# if our score does not have sufficient probability,
			# ignore it
			if scoresData[x] < 0.5:
				continue
			# compute the offset factor as our resulting feature
			# maps will be 4x smaller than the input image
			(offsetX, offsetY) = (x * 4.0, y * 4.0)
			# extract the rotation angle for the prediction and
			# then compute the sin and cosine
			angle = anglesData[x]
			cos = np.cos(angle)
			sin = np.sin(angle)
			# use the geometry volume to derive the width and height
			# of the bounding box
			h = xData0[x] + xData2[x]
			w = xData1[x] + xData3[x]
			# compute both the starting and ending (x, y)-coordinates
			# for the text prediction bounding box
			endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
			endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
			startX = int(endX - w)
			startY = int(endY - h)
			# add the bounding box coordinates and probability score
			# to our respective lists
			rects.append((startX, startY, endX, endY))
			confidences.append(scoresData[x])
	# return a tuple of the bounding boxes and associated confidences
	return (rects, confidences)


def vertical_control(distance, height):
	return min((distance **2 - 0.25) , 0.0) * min(1.0, height + 0.5) # * (height**2 + 0.2)


class PID():
	def __init__(self, kp, ki, kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd

		self.previous_error = np.zeros(2)
		self.integration = np.zeros(2)
		print(f"PID created: kp = {self.kp}; ki = {self.ki}; kd = {self.kd}")
	
	def evaluate_pid(self, vector_target):

		error = vector_target #/np.linalg.norm(vector_target)

		prop = self.kp * error
		integ = self.ki * self.integration
		deriv = self.kd * (error - self.previous_error)

		print(f'Controller: {np.linalg.norm(prop)}; {np.linalg.norm(integ)}; {np.linalg.norm(deriv)}')

		control = prop + integ + deriv

		self.previous_error = error

		self.actualize_integration(error)

		return control
	
	def actualize_integration(self, error):
		limitation = 30000
		if np.linalg.norm(self.integration + error) > limitation:
			self.integration = (self.integration + error) * (limitation/np.linalg.norm(self.integration + error))
		else:
			self.integration += error




##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################
class DroneNode(Node): 
	def __init__(self, vehicle):
		super().__init__('drone_node')  # This initializes the ROS 2 node
		self.vehicle = vehicle

		self.configure_vehicle()

		self.get_logger().info("Drone node's up tutututututtu")

		self.land_id_to_find = 1 
		# self.vehicle.parameters['PLND_ENABLED']=0
		# self.vehicle.parameters["RNGFND1_TYPE"] = 0
		self.takeoff_height = 15

		# aruco dict 
		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		self.parameters = aruco.DetectorParameters()
		self.arucoDetector = aruco.ArucoDetector(self.aruco_dict, self.parameters)


		# camera details
		self.horizontal_res = 640 
		self.vertical_res = 480 
		self.aspect_ratio = self.vertical_res/self.horizontal_res
		self.fov = 1.085
		self.angle_per_pixel_h = self.fov/self.horizontal_res
		self.angle_per_pixel_v = self.angle_per_pixel_h  * self.aspect_ratio
		self.horizontal_fov = 62.2 * (math.pi / 180)  # convert to radians 
		self.vertical_fov = 48.8 * (math.pi / 180)  # convert to radians
		self.dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0]  # from the rostopic camera info
		self.camera_matrix = [[530.8269276712998, 0.0, 320.5],[0.0, 530.8269276712998, 240.5],[0.0, 0.0, 1.0]]
		self.np_camera_matrix = np.array(self.camera_matrix)
		self.np_dist_coeff = np.array(self.dist_coeff)

		self.found_count = 0 
		self.notfound_count = 0  # keep track of aruco markers

		self.waypoints = []

		# keep track of time for update on precision landing
		self.time_last = 0.0
		self.time_to_wait = 0.1


		# image info
		qos_newing = QoSProfile(depth=10)
		self.newimg_pub = self.create_publisher(Image, '/camera/color/image_new', qos_newing)
		self.marker_size = 20
		qos_procimg = QoSProfile(depth=10)
		self.procimg_pub = self.create_publisher(Image, '/camera/color/processed_image', qos_procimg)

		self.drone_status_pub = self.create_publisher(String, '/drone_status', 10)

		qos_profile = QoSProfile(depth=10)
		# self.video_sub = self.create_subscription(Image, '/camera/image_raw', self.video_feed_processing, qos_profile)
		self.command_subscription = self.create_subscription(String, '/drone_command', self.command_callback, 10)

		self.waypoint_sub = self.create_subscription(PoseArray, '/drone/waypoint', self.waypoints_callback, 10)		
		self.waypoint_sub  # prevent unused variable warning

		self.marker_position_sub = self.create_subscription(Point, '/drone/target_marker_position', self.marker_position_callback, 10)

		self.yaw_setup_sub = self.create_subscription(String,
            '/drone/marker_35_detection',
			self.marker_35_callback, # Only used at the initialisation to setup the coordinate transformation
            10)
		
		self.yaw_transform_setup = False

		self.home_position_pub = self.create_publisher(String, '/home_position', 10)

		self.bridge = CvBridge()

		self.home_gps = self.vehicle.location.global_relative_frame
		self.lat_home = self.home_gps.lat
		self.lon_home = self.home_gps.lon


		self.has_taken_off = False
		self.search_ongoing = False
		self.landing_allowed = False
		self.follow_marker = False
		# self.target_marker_found = False
		self.bounds = {'north': 20, 'south': -20, 'east': 15, 'west': -15}
		self.tag_positions = {}
		self.currently_seen_markers = []
		self.waypoint_spacing = 5
		self.targeted_aruco_marker_relative_coordinates = None
		self.precision_landing = False
		self.count = 0

		self.flight_data = []


		self.marker_pid = PID(0.25, 0.03, 0.25)

	def marker_35_callback(self, msg=String()):
		if not self.yaw_transform_setup:
			marker = json.loads(msg.data)
			self.get_logger().info(f'35 data {marker}')
			marker_id = marker['id']
			if marker_id == 35:
				# Calculate the position offsets from the drone to the marker
				x = float(marker['position']['x']) - (self.horizontal_res / 2)  # Assuming center of image as origin
				y = float(marker['position']['y']) - (self.vertical_res / 2)  # Assuming center of image as origin

				self.yaw_transform_setup = True
		
				self.yaw_correction = math.atan2(y, x)

				self.get_logger().info(f'yaw correction = {self.yaw_correction}')

	
	def gps_to_local_north_east(self, lat, lon):
		"""
		Convert GPS coordinates to local north, east coordinates relative to a reference point.
		lat1, lon1: Reference latitude and longitude.
		lat2, lon2: Target latitude and longitude.
		Returns: north (meters), east (meters)
		"""
		# Earth's radius in meters
		R = 6371000

		# Degree to radians conversion
		def to_rad(degrees):
			return degrees * math.pi / 180

		dLat = to_rad(lat - self.lat_home)
		dLon = to_rad(lon - self.lon_home)

		north = dLat * R
		east = dLon * R * math.cos(to_rad(self.lat_home))

		return np.array((north, east))
	
	def compute_horizontal_distance(self, vector):
		angle_x = vector[0] * self.angle_per_pixel_h
		angle_y = vector[1] * self.angle_per_pixel_v

		return np.linalg.norm(self.vehicle.location.global_relative_frame.alt * np.array([math.tan(angle_x), math.tan(angle_y)]))
	
	def compute_horizontal_vector_meter(self, vector):
		angle_x = vector[0] * self.angle_per_pixel_h
		angle_y = vector[1] * self.angle_per_pixel_v

		return self.vehicle.location.global_relative_frame.alt * np.array([math.tan(angle_x), math.tan(angle_y)])
	
		
	def marker_global_coordinates(self):
		theta =  -self.vehicle.attitude.yaw - self.yaw_correction
		self.get_logger().info(f'yaw = {self.vehicle.attitude.yaw}')
		rot_mat = np.array([
			[math.cos(theta), math.sin(theta)],
			[-math.sin(theta), math.cos(theta)]
		])

		return -rot_mat.dot(self.targeted_aruco_marker_relative_coordinates)

	def publish_home_position(self):
		home_position = {'lat': self.lat_home, 'lon': self.lon_home}
		home_position_msg = String()
		home_position_msg.data = json.dumps(home_position)
		self.home_position_pub.publish(home_position_msg)
	

	def command_callback(self, msg):
		command = msg.data
		self.get_logger().info(f'Received command: {command}')
		
		if "takeoff" in command:
			self.publish_home_position()
			# Extract altitude from the command string
			altitude = float(command.split("; altitude: ")[1])
			self.arm_and_takeoff(altitude)
			
		elif "start_grid_search" in command:
			self.start_grid_search()

		elif "clear_command_line" in command:
			self.clear_command_line()
			self.vehicle.mode = VehicleMode("GUIDED")
			while self.vehicle.mode != "GUIDED":
				print('Setting vehicle to GUIDED mode...')
				time.sleep(0.5)
			
		elif "land_on_marker" in command:
			self.follow_marker = True
			
		elif "initiate_landing" in command:
			self.initiate_landing()
			
		else:
			self.get_logger().info("Received unknown command.")
	

	def clear_command_line(self):
		cmds = self.vehicle.commands
		cmds.download()
		cmds.wait_ready()
		cmds.clear()
	
	def waypoints_callback(self, msg):
		self.waypoints = []
		for pose in msg.poses:
			lat = pose.position.x  # Extract latitude
			lon = pose.position.y  # Extract longitude
			self.waypoints.append((lat, lon))
		# Now, waypoints list contains all the waypoints as (latitude, longitude) tuples
		# You can process the waypoints list as needed
		print("Received waypoints:", self.waypoints)


	def get_location_metres(self, original_location, dNorth, dEast):
		"""
		Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
		specified `original_location`. The returned LocationGlobal has the same `alt` value as `original_location`.
		"""
		earth_radius = 6378137.0  # Radius of "spherical" earth
		# Coordinate offsets in radians
		dLat = dNorth / earth_radius
		dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

		# New location
		newlat = original_location.lat + (dLat * 180/math.pi)
		newlon = original_location.lon + (dLon * 180/math.pi)
		return LocationGlobalRelative(newlat, newlon, original_location.alt)
	

	def get_distance_meters(self, aLocation1, aLocation2):
		"""
		Calculates the ground distance in meters between two LocationGlobal objects.

		:param aLocation1: LocationGlobal object of the first point.
		:param aLocation2: LocationGlobal object of the second point.
		:return: Distance in meters.
		"""
		dlat = aLocation2.lat - aLocation1.lat
		dlong = aLocation2.lon - aLocation1.lon
		return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


	def send_ned_velocity(self, vx, vy, vz=0):
		"""
		Send velocity command to the drone in the local frame (NED - North East Down).
		"""
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target_system, target_component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
			0b0000111111000111,  # type_mask (only velocities)
			0, 0, 0,  # position (not used)
			vx, vy, vz,  # velocity
			0, 0, 0,  # acceleration (not used)
			0, 0)     # yaw, yaw_rate (not used)

		# Send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()


	def marker_position_callback(self, msg):
		# Assuming the drone uses these coordinates directly for navigation or further processing

		self.targeted_aruco_marker_relative_coordinates = np.array([msg.x, msg.y])
		distance = self.compute_horizontal_distance(self.targeted_aruco_marker_relative_coordinates)

		if self.follow_marker:
			self.count += 1
			self.get_logger().info(f'Here s the count: {self.count}')
			self.get_logger().info(f"Drone Global Position {self.gps_to_local_north_east(self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon)}")
			self.get_logger().info(f"Marker position{self.targeted_aruco_marker_relative_coordinates}")
			delta_vector = self.vehicle.location.global_relative_frame.alt * np.array((math.tan(self.vehicle.attitude.roll), math.tan(self.vehicle.attitude.pitch)))
			print(f'correction {delta_vector}')
			true_vector = self.compute_horizontal_vector_meter(self.marker_global_coordinates()) #- delta_vector
			print(f'true vector {true_vector}; altitude  {self.vehicle.location.global_relative_frame.alt}')
			control = self.marker_pid.evaluate_pid(true_vector)

			if self.precision_landing:
				if self.vehicle.mode.name != 'LAND':
					self.vehicle.mode = VehicleMode('LAND')
				self.get_logger().info(f'Precision landing at altitude {self.vehicle.location.global_relative_frame.alt}')
				
				x, y = true_vector[0], true_vector[1]
				self.send_land_message(x, y)

			else:
				velocity_x, velocity_y = control[0], control[1]
				velocity_z = -vertical_control(distance, self.vehicle.location.global_relative_frame.alt) # ffffffffffffffffffffffff
				self.get_logger().info(f"Received velocity command{velocity_x, velocity_y, velocity_z}")
				self.get_logger().info(f"Distance{distance}")
				self.send_ned_velocity(velocity_x, velocity_y, velocity_z)
				self.compute_horizontal_distance(self.targeted_aruco_marker_relative_coordinates)
				if self.vehicle.location.global_relative_frame.alt < 0.2:
					#self.precision_landing = True
					self.vehicle.mode = VehicleMode('LAND')
				# 	x, y = true_vector[0], true_vector[1]
				# 	self.send_land_message(x, y)
				time.sleep(0.1)
				current_time = self.get_clock().now().to_msg()  # Get the current time
				data_point = {
					"time": str(current_time),
					"altitude": self.vehicle.location.global_relative_frame.alt,
					"distance": distance,
					"control_x": control[0],
					"control_y": control[1],
					"control_z": -vertical_control(distance, self.vehicle.location.global_relative_frame.alt)  # Example for control_z
				}
				self.flight_data.append(data_point)


	def send_land_message(self, x, y): 
		# for precision landing
	# Convert x and y distances to angles in radians
		angle_x = math.atan2(y, x)  # y first if y is the forward direction
		angle_y = math.atan2(x, y)  # x first if x is the forward direction
		distance = math.sqrt(x**2 + y**2)

		msg = self.vehicle.message_factory.landing_target_encode(
				0,  # time_usec (not used)
				0,  # target_num (not used)
				mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
				angle_x,  # X-axis angular offset (radians)
				angle_y,  # Y-axis angular offset (radians)
				distance,  # Distance, or 0 if unknown
				0, 0)  # Size of the landing area, if known
		
		self.get_logger().info(f'landing message: {msg}')
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

	def save_flight_data(self, file_path):
		with open(file_path, 'w') as f:
			json.dump(self.flight_data, f, indent=4)

	def pixel_to_meters(self, x_offset, y_offset, altitude):
		"""
		Converts pixel offsets to meters based on drone altitude and camera calibration.
		Placeholder for actual conversion logic.
		"""
		# Placeholder conversion factors; adjust based on your camera and typical altitudes
		scale_x = altitude * 0.001
		scale_y = altitude * 0.001
		
		return x_offset * scale_x, y_offset * scale_y
	
	def configure_vehicle(self):
		self.vehicle.parameters['PLND_ENABLED'] = 1
		self.vehicle.parameters['PLND_TYPE'] = 1
		self.vehicle.parameters['PLND_EST_TYPE'] = 0
		self.vehicle.parameters['LAND_SPEED'] = 20  # cm/s

		self.vehicle.parameters["RNGFND1_TYPE"] = 1
		self.vehicle.parameters["RNGFND1_MIN_CM"] = 0
		self.vehicle.parameters["RNGFND1_MAX_CM"] = 400
		self.vehicle.parameters["RNGFND1_PIN"] = 0
		self.vehicle.parameters["RNGFND1_SCALING"] = 12.12
		self.vehicle.parameters['WPNAV_SPEED'] = 500.0  # cm/s

	def arm(self):
		while self.vehicle.is_armable!=True:
			print("Waiting for vehicle to become armable.")
			time.sleep(1)
		print("Vehicle is now armable")
	    
		self.vehicle.mode = VehicleMode("GUIDED")
	            
		while self.vehicle.mode!='GUIDED':
			print("Waiting for drone to enter GUIDED flight mode")
			time.sleep(1)
		print("Vehicle now in GUIDED MODE. Have fun!!")

		self.vehicle.armed = True
		while self.vehicle.armed==False:
			print("Waiting for vehicle to become armed.")
			time.sleep(1)
			print("Look out! Virtual props are spinning!!")
			time.sleep(.5)

		return None
	
	def initiate_landing(self):
		"""
		Initiates the landing sequence for the drone.
		"""
		self.get_logger().info('Initiating landing sequence...')
		# Example: Switch to LAND mode to start landing.
		# This is a placeholder; your actual landing logic may differ.
		self.vehicle.mode = VehicleMode('LAND')
		while not self.vehicle.location.global_relative_frame.alt <= 0.1:  # Check altitude is near 0
			self.get_logger().info(f'Current altitude: {self.vehicle.location.global_relative_frame.alt}')
			time.sleep(1)
		self.get_logger().info('Landing complete.')

	def arm_and_takeoff(self, altitude=None):
		self.get_logger().info(f'Received command to take off at: {altitude}')

		if altitude is None:
			altitude = self.takeoff_height

		while self.vehicle.is_armable != True:
			print('Waiting for vehicle to become armable')
			time.sleep(1)

		print('vehicle is now armable')
		self.vehicle.mode = VehicleMode('GUIDED')

		while self.vehicle.mode != "GUIDED":
			print('Waiting for drone to enter guided flight mode')
			time.sleep(1)

		print('Vehicle now in guided mode')

		self.vehicle.armed = True 
		while self.vehicle.armed == False:
			print('Waiting for vehicle to become armed')
			time.sleep(1)

		print("props are spinning")

		self.get_logger().info(f"Taking off to {altitude} meters.")

		self.vehicle.simple_takeoff(self.takeoff_height)  # meters 

		while True:
			print("Current Altitude: %d"%self.vehicle.location.global_relative_frame.alt)
			if self.vehicle.location.global_relative_frame.alt >= self.takeoff_height*0.95:
				break 
			time.sleep(1)

		print("target altitude reached")
		self.has_taken_off = True

		msg = String()
		msg.data = "has taken off"
		self.drone_status_pub.publish(msg)

		return None 
	
	def start_grid_search(self):
		self.search_ongoing = True
		cmds = self.vehicle.commands
		cmds.download()
		cmds.wait_ready()
		cmds.clear()
		print("Configuring waypoints for grid search...")
		for waypoint in self.waypoints:
			lat, lon = waypoint
			cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, self.takeoff_height)
			cmds.add(cmd)

		self.vehicle.commands.upload()

		self.arm_and_takeoff()

		self.vehicle.mode = VehicleMode("AUTO")
		while self.vehicle.mode != "AUTO":
			time.sleep(0.2)



		# # Once the target marker is found, clear the waypoints and prepare for landing
		# if self.target_marker_found.is_set():
		# 	self.vehicle.commands.clear()
		# 	self.vehicle.commands.upload()
		# 	print("Target marker found, preparing for precision landing...")
		# 	# Transition to precision landing behavior
		# 	self.search_ongoing = False
		# 	self.precision_unmoving_landing_enabled = True

		# 	newlat, newlon = self.get_global_position_of_marker()

		# 	print('aaaaaaaaaaa', newlat, newlon)

		# 	cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, newlat, newlon, self.takeoff_height)
		# 	cmds.add(cmd)
		# 	self.vehicle.commands.upload()

		# 	self.vehicle.mode = VehicleMode("AUTO")
		# 	while self.vehicle.mode != "AUTO":
		# 		time.sleep(0.2)

		# 	while np.linalg.norm(self.targeted_aruco_marker_relative_coordinates) > 0.1:
		# 		time.sleep(0.1)
		# 		print(self.targeted_aruco_marker_relative_coordinates)

		# 	self.initiate_landing()
	
	

	def goto(self, targetLocation):
		
		# input is a LocationGlobalRelative variable
		distanceToTargetLocation = get_distance_meters(targetLocation, self.vehicle.location.global_relative_frame)

		self.vehicle.simple_goto(targetLocation, 10)

		while self.vehicle.mode.name == "GUIDED":
			print('Mode = ', self.vehicle.mode.name)
			currentDistance = get_distance_meters(targetLocation, self.vehicle.location.global_relative_frame)
			print(abs(100*currentDistance/distanceToTargetLocation))
			if currentDistance < distanceToTargetLocation*0.1:
				print("Reached target waypoint")
				time.sleep(2)
				break 
			time.sleep(1)

		return None

	def goto_position_target_local_ned(self,north, east, down):
		"""
	    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
	    location in the North, East, Down frame.
	    """
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
	    	0,       # time_boot_ms (not used)
		    0, 0,    # target system, target component
	        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
	        0b0000111111111000, # type_mask (only positions enabled)
	        north, east, down,
	        0, 0, 0, # x, y, z velocity in m/s  (not used)
	        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
	        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	    # send command to vehicle
		self.vehicle.send_mavlink(msg)

	
	def return_and_land(self):
		"""
		Directs the drone to return to its home position and land.
		"""
		self.get_logger().info('Returning to launch location and landing...')

		# Check if vehicle is in a mode that allows changing position
		if self.vehicle.mode.name not in ['GUIDED', 'AUTO']:
			self.get_logger().info('Switching to GUIDED mode for return journey.')
			self.vehicle.mode = VehicleMode('GUIDED')
			# Wait for mode to change
			while not self.vehicle.mode.name == 'GUIDED':
				time.sleep(0.5)

		# Return to launch (RTL) mode automatically returns the vehicle to its home position and lands it
		self.vehicle.mode = VehicleMode('RTL')

		# Wait for the vehicle to land (check altitude)
		while self.vehicle.location.global_relative_frame.alt > 0.1:  # Wait until the altitude is less than 0.1 meters
			self.get_logger().info(f'Current altitude: {self.vehicle.location.global_relative_frame.alt}')
			time.sleep(1)

		self.get_logger().info('Landing complete.')


	def condition_yaw(self, degrees, relative):
		if relative:
			is_relative = 1  # yaw relative to direction of travel 

		else:
			is_relative = 0  # yaw is absolute angle 

		# create the CONDITION_YAW command using command_long_encode()
		msg = self.vehicle.message_factory.command_long_encode(
			0, 0,  # target system, target component 
			mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command 
			0,  # confirmation 
			degrees,  # # param 1, yaw in degrees 
			0,  # param 2, yaw speed deg/s
			1,  # param 3, direction -1 ccw, 1 cw 
			is_relative,  # param 4, relative offset 1, absolute angle 0 
			0, 0, 0)  # param 5-7 not used 

		# send command to vehicle 
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush() 


	# fly to waypoint. we use ti as a dummy function to setup the condition_yaw function 
	def dummy_yaw_initializer(self):
		lat = self.vehicle.location.global_relative_frame.lat
		lon = self.vehicle.location.global_relative_frame.lon  
		alt = self.vehicle.location.global_relative_frame.alt  

		aLocation=LocationGlobalRelative(lat, lon, alt)
		msg = self.vehicle.message_factory.set_position_target_global_int_encode(
			0,  # time_boot_ms (not used)
			0,0,  # target system, target component
			mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame  
			0b0000111111000111,  # .. BITMASK -> consider only the velocities 
			aLocation.lat*1e7,  # lat int - x position in wgs84 frame in 1e7 meters 
			aLocation.lon*1e7,  # lon int - y position in wgs84 frame in 1e7 meters  
			aLocation.alt,  # alt - altitude in meters 
			0, 0, 0,  # velocity 
			0, 0, 0,  # acceleration
			0, 0)

		# send command to vehicle 
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush() 

	def land(self):
		self.vehicle.parameters['PLND_ENABLED']=1
		self.vehicle.parameters['PLND_TYPE']=1
		self.vehicle.parameters['PLND_TYPE']=3
		self.vehicle.parameters['PLND_EST_TYPE']=0
		self.vehicle.parameters['LAND_SPEED']= 30 # cm/s 

		# set the rangefinder parameters 
		self.vehicle.parameters["RNGFND1_TYPE"] = 1
		self.vehicle.parameters["RNGFND1_MIN_CM"] = 0 
		self.vehicle.parameters["RNGFND1_MAX_CM"] = 1500 
		self.vehicle.parameters["RNGFND1_PIN"] = 0  
		self.vehicle.parameters["RNGFND1_SCALING"] = 12.12 
		self.vehicle.mode = VehicleMode('LAND')

		while self.vehicle.mode != "LAND":
			time.sleep(0.5)


	# #### PRECISION LANDING ######
	def msg_receiver(self, message):
		# global notfound_count, found_count, time_last, time_to_wait, id_to_find


		self.vehicle.mode = VehicleMode('LAND')
		while self.vehicle.mode != 'LAND':
			self.vehicle.mode = VehicleMode('LAND')
			time.sleep(1.0)

		if time.time() - self.time_last > self.time_to_wait:
			np_data = rnp.numpify(message)  # deserialize image data into array 
			gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

			ids = ''
			(corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=self.aruco_dict, parameters=self.parameters)

			try:
				if ids is not None:
					if ids[0]==self.land_id_to_find:
						ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, cameraMatrix=self.np_camera_matrix,
							distCoeffs=self.np_dist_coeff)
						(rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])  # rotation vctr and translation vctr
						x = '{:.2f}'.format(tvec[0])  # x error 
						y = '{:.2f}'.format(tvec[1])
						z = '{:.2f}'.format(tvec[2])

						y_sum = 0
						x_sum = 0 

						x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
						y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
						
						x_avg = x_sum /4 
						y_avg = y_sum /4 

						# two values required for the precision landing
						x_ang = (x_avg - self.horizontal_res*0.5)*self.horizontal_fov/self.horizontal_res
						y_ang = (y_avg - self.vertical_res*0.5)*self.vertical_fov/self.vertical_res

						if self.vehicle.mode !="LAND":
							self.vehicle.mode = VehicleMode("LAND")
							while self.vehicle.mode != "LAND":
								time.sleep(0.1)
							print("Vehicle is in LAND mode")
							self.send_land_message(x_ang, y_ang)
						else:
							self.send_land_message(x_ang, y_ang)

						marker_position = 'Marker Position: x='+x+' y='+y+' z='+z+''

						aruco.drawDetectedMarkers(np_data, corners)
						aruco.drawAxis(np_data, self.np_camera_matrix, self.np_dist_coeff, rvec, tvec, 10)  
						cv2.putText(np_data, marker_position, (10,50), 0, 0.7, (255, 0,0), thickness=2)
						print(marker_position)
						print('FOUND COUNT:' + str(self.found_count) + " NOT FOUND COUNT:" + str(self.notfound_count))
						self.found_count += 1 
					else: 
						self.notfound_count +=1 
						print("Target Not Found")
				else: 
					self.notfound_count +=1 
			except Exception as e: 
				print("Target not found")
				print(e)
				self.notfound_count += 1

			new_msg = rnp.msgify(Image, np_data, encoding="rgb8")
			self.newimg_pub.publish(new_msg)
			self.camera_pub.publish(new_msg)
			self.time_last = time.time()

		else:
			return None


	
	def pl_msg_receiver_landing_pad(self, message):
		if not self.landing_allowed:
			pass
		
		else:
			self.vehicle.mode = VehicleMode('LAND')

			while self.vehicle.mode != 'LAND':
				self.vehicle.mode = VehicleMode('LAND')
				time.sleep(1.0)

			if time.time() - self.time_last > self.time_to_wait:
				self.get_logger().info('lander is analyzing')
				bridge = CvBridge()
				np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
				gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

				cv2.imshow('Window Title', gray_img)
				cv2.waitKey(1)

				flipped = cv2.flip(gray_img, 1)
				corners, ids, rejectedImgPoints = self.arucoDetector.detectMarkers(gray_img)



				for i, corner in zip(ids, corners):
					if self.targeted_aruco_marker == i[0]:
						self.adjust_for_landing(corner)


				# Publish the processed image
				new_msg = bridge.cv2_to_imgmsg(np_data, encoding="rgb8")
				self.newimg_pub.publish(new_msg)
				self.time_last = time.time()
			else:
				return None

	

	def find_letter(self, letter, message):
	
		if time.time() - self.time_last > self.time_to_wait and self.search_ongoing == True:
			### the old way ###
			# np_data = rnp.numpify(message)  # deserialize image data into array 

			### try cvbridge 
			bridge = CvBridge()
			np_data = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
			# gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
			# gray_img = get_grayscale(np_data)

			# change the image to remove noise 
			image = np_data
			orig = image.copy()
			(origH, origW) = image.shape[:2]
			# set the new width and height and then determine the ratio in change
			# for both the width and height
			(newW, newH) = (320, 320)
			rW = origW / float(newW)
			rH = origH / float(newH)
			# resize the image and grab the new image dimensions
			image = cv2.resize(image, (newW, newH))
			(H, W) = image.shape[:2]

			# define the two output layer names for the EAST detector model that
			# we are interested in -- the first is the output probabilities and the
			# second can be used to derive the bounding box coordinates of text
			layerNames = [
				"feature_fusion/Conv_7/Sigmoid",
				"feature_fusion/concat_3"]
			# load the pre-trained EAST text detector
			print("[INFO] loading EAST text detector...")
			net = cv2.dnn.readNet("frozen_east_text_detection.pb")

			# construct a blob from the image and then perform a forward pass of
			# the model to obtain the two output layer sets
			blob = cv2.dnn.blobFromImage(image, 1.0, (W, H),
				(123.68, 116.78, 103.94), swapRB=True, crop=False)
			net.setInput(blob)
			(scores, geometry) = net.forward(layerNames)
			# decode the predictions, then  apply non-maxima suppression to
			# suppress weak, overlapping bounding boxes
			(rects, confidences) = decode_predictions(scores, geometry)
			boxes = non_max_suppression(np.array(rects), probs=confidences)

			# initialize the list of results
			results = []
			# loop over the bounding boxes
			for (startX, startY, endX, endY) in boxes:
				# scale the bounding box coordinates based on the respective
				# ratios
				startX = int(startX * rW)
				startY = int(startY * rH)
				endX = int(endX * rW)
				endY = int(endY * rH)
				# in order to obtain a better OCR of the text we can potentially
				# apply a bit of padding surrounding the bounding box -- here we
				# are computing the deltas in both the x and y directions
				padding = 0.0
				dX = int((endX - startX) * padding)
				dY = int((endY - startY) * padding)
				# apply padding to each side of the bounding box, respectively
				startX = max(0, startX - dX)
				startY = max(0, startY - dY)
				endX = min(origW, endX + (dX * 2))
				endY = min(origH, endY + (dY * 2))
				# extract the actual padded ROIpytesseract
				# (3) an OEM value, in this case, 7 which implies that we are
				# treating the ROI as a single line of text
				roi = get_grayscale(roi)
				roi = thresholding(roi)
				roi = noise_removal(cv2.bitwise_not(roi))
				roi = thin_font(roi)
				config = ("-l eng --oem 1 --psm 10")
				text = pytesseract.image_to_string(roi, config=config)
				# add the bounding box coordinates and OCR'd text to the list
				# of results
				if letter in text:
					results.append(((startX, startY, endX, endY), text))

			# sort the results bounding box coordinates from top to bottom
			results = sorted(results, key=lambda r:r[0][1])
			# loop over the results
			for ((startX, startY, endX, endY), text) in results:
				# display the text OCR'd by Tesseract
				# print("OCR TEXT")
				# print("========")
				# print("{}\n".format(text))
				# strip out non-ASCII text so we can draw the text on the image
				# using OpenCV, then draw the text and a bounding box surrounding
				# the text region of the input image
				text = "".join([c if ord(c) < 128 else "" for c in text]).strip()

			return text, startX, startY, endX, endY, H, W 
	
	def center_letter(self, letter, message):
		# this function is designed to take a target point from the an image and get the drone center on the target location
		centered = False 

		while not centered:
			text, startX, startY, endX, endY, H, W = self.find_letter(letter, message)

			centerX = (startX + endX) / 2 
			centerY = (startY + endY) / 2 

			gps_location = []
			if 0.95*W/2 <= centerX <= 1.05*W/2 and 0.95*H/2 <= centerY <= 1.05*H/2:
				centered = True 
				return None 
			
	
	def adjust_for_landing(self, corner):
		"""
		Calculate position adjustments based on marker's position and initiate landing.
		"""
		# Calculate marker's center
		centerX = int(sum([c[0] for c in corner[0]]) / 4)
		centerY = int(sum([c[1] for c in corner[0]]) / 4)

		# Calculate offsets from the center of the image
		x_offset = (centerX - self.horizontal_res / 2) * self.horizontal_fov / self.horizontal_res
		y_offset = (centerY - self.vertical_res / 2) * self.vertical_fov / self.vertical_res

		# Adjust position based on the offsets
		self.send_land_message(x_offset, y_offset)

		
	
	# def search(self):
	# 	"""A function to search the field for the targets"""
	# 	pass
	
	def precision_land_landing_pad(self):
		print('Landing...')
		qos_profile = QoSProfile(depth=10)
		# self.landing_sub = self.create_subscription(Image, '/camera/image_raw', self.pl_msg_receiver_landing_pad, qos_profile)
		while self.vehicle.armed == True:
			time.sleep(1)
		return None 
		





def main():
	rclpy.init()
	vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
	drone = DroneNode(vehicle)

	try:
		rclpy.spin(drone)
	finally:
		file_path = '/home/hugues/robotx_ws/simulations/results/flight_data.json' 
		drone.save_flight_data(file_path)
		drone.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
    main()

    