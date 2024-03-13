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
import threading
from geometry_msgs.msg import PoseArray
# from robotx.msg import MissionState
# from robotx.action import DroneOperation  # Replace with your actual package name and action file name


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



##################  GENERIC FUNCTIONS FOR BASIC MOVEMENT ################
class Drone(Node): 
	def __init__(self, vehicle):
		super().__init__('drone_node')  # This initializes the ROS 2 node
		self.vehicle = vehicle

		self.configure_vehicle()

		self.land_id_to_find = 72 
		# self.vehicle.parameters['PLND_ENABLED']=0
		# self.vehicle.parameters["RNGFND1_TYPE"] = 0
		self.takeoff_height = 10

		# aruco dict 
		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
		self.parameters = aruco.DetectorParameters()
		self.arucoDetector = aruco.ArucoDetector(self.aruco_dict, self.parameters)


		# camera details
		self.horizontal_res = 640 
		self.vertical_res = 480 
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
		self.mission_service = self.create_service(Trigger, 'start_mission2', self.handle_mission_request)
		
		self.camera_sub = self.create_subscription(String, '/drone/marker_detection', self.callback_marker, 10)

		self.waypoint_sub = self.create_subscription(
            PoseArray,  # Message type
            '/drone/waypoint',  # Topic
            self.waypoint_callback,  # Callback function
            10)  # QoS profile
		
		self.waypoint_sub  # prevent unused variable warning

		# self._action_server = ActionServer(
		# 	self,
		# 	DroneOperation,
		# 	'drone_command',
		# 	self.execute_callback)

		# qos_camera = QoSProfile(depth=10)
		# self.camera_pub  = self.create_publisher(Image, '/camera/color/image_raw', qos_camera)

		self.bridge = CvBridge()

		self.home_gps = self.vehicle.location.global_relative_frame
		self.lat_home = self.home_gps.lat
		self.lon_home = self.home_gps.lon

		self.has_taken_off = False
		self.search_ongoing = False
		self.landing_allowed = False
		# self.target_marker_found = False
		self.target_marker_found = threading.Event()
		self.bounds = {'north': 20, 'south': -20, 'east': 15, 'west': -15}
		self.tag_positions = {}
		self.currently_seen_markers = []
		self.targeted_aruco_marker = 72
		self.waypoint_spacing = 5
		self.targeted_aruco_marker_relative_coordinates = None

		# async def execute_callback(self, goal_handle):
		# 	feedback_msg = DroneOperation.Feedback()
		# 	result = DroneOperation.Result()

		# 	# Process the goal command
		# 	command = goal_handle.request.command
		# 	self.get_logger().info(f'Executing command: {command}')

		# 	# Example: Handle different commands
		# 	if command == 'takeoff':
		# 		# Perform takeoff
		# 		self.arm_and_takeoff(5)  # Assuming this is an async function or modify to suit
		# 		feedback_msg.feedback = 'Taking off'
		# 		goal_handle.publish_feedback(feedback_msg)
		# 		...
		# 	elif command == 'start_grid_search':
		# 		# Perform grid search
		# 		...

		# 	# Once command execution is complete
		# 	result.success = True
		# 	result.message = f'Command {command} executed successfully'
		# 	return result
	
            
	def mission2_land_on_unmoving_target_async(self):
		# Wrap the original function to run in a separate thread
		thread = threading.Thread(target=self.mission2_land_on_unmoving_target)
		thread.start()

	def mission2_land_on_unmoving_target(self):

		self.arm_and_takeoff()

		self.waypoints = self.generate_grid_search_waypoints()

		self.start_grid_search()
		
		if not self.target_marker_found.is_set():
			self.return_and_land()

	
	def callback_marker(self, msg):
		if self.search_ongoing:
			self.handle_marker_detection(msg)
		elif self.target_marker_found.is_set():
			marker_data = json.loads(msg.data)
			for marker in marker_data:
				if marker['id'] == self.targeted_aruco_marker:
					# Calculate the position offsets from the drone to the marker
					x_offset = marker['position']['x'] - (self.horizontal_res / 2)  # Assuming center of image as origin
					y_offset = marker['position']['y'] - (self.vertical_res / 2)  # Assuming center of image as origin
					self.targeted_aruco_marker_relative_coordinates = np.array([x_offset, y_offset])

	def get_global_position_of_marker(self, relative_position=None):
		"""
		Calculate the global position of the marker based on its relative position to the drone
		and the drone's global position.

		:param relative_position: The relative position of the marker to the drone (x, y, z).
		:return: Global position of the marker (latitude, longitude, altitude).
		"""
		if relative_position is None:
			while self.targeted_aruco_marker_relative_coordinates is None:
				self.get_logger().error("Relative position of the marker is not available yet.")
				time.sleep(0.5)
			relative_position = self.targeted_aruco_marker_relative_coordinates

		# Assuming self.vehicle.location.global_relative_frame gives us the global position
		current_location = self.vehicle.location.global_relative_frame
		bearing = math.atan2(relative_position[1], relative_position[0])
		distance = math.sqrt(relative_position[0] ** 2 + relative_position[1] ** 2)  # Distance in meters

		# Constants
		earth_radius = 6371000  # Earth's radius in meters

		# Calculate new latitude
		delta_latitude = distance * math.cos(bearing) / earth_radius
		new_latitude = current_location.lat + math.degrees(delta_latitude)

		# Calculate new longitude
		delta_longitude = distance * math.sin(bearing) / (earth_radius * math.cos(math.radians(new_latitude)))
		new_longitude = current_location.lon + math.degrees(delta_longitude)

		return new_latitude, new_longitude
	

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
	


	# def precision_unmoving_landing(self):
	# 	"""
	# 	Perform a precision landing on an unmoving target using relative position vectors.
	# 	"""
	# 	# Ensure the drone has taken off and is in GUIDED mode
	# 	self.vehicle.mode = VehicleMode("GUIDED")
	# 	while self.vehicle.mode != VehicleMode("GUIDED"):
	# 		time.sleep(0.5)
	# 		self.get_logger().info("Drone must be airborne and in GUIDED mode to perform precision landing.")

	# 	# Wait for the target marker's relative position to be detected
	# 	while self.targeted_aruco_marker_relative_coordinates is None:
	# 		self.get_logger().info("Waiting for target marker detection...")
	# 		time.sleep(1.0)

	# 	# Continuously adjust drone position based on relative position to marker
	# 	while not self.vehicle.location.global_relative_frame.alt <= 0.01:  # Arbitrary low altitude check for landing completion
	# 		# Calculate the required velocity vector to move towards the marker
	# 		x_offset, y_offset = self.targeted_aruco_marker_relative_coordinates
	# 		# Convert pixel offsets to meters - this may require tuning based on your setup
	# 		x_offset_meters, y_offset_meters = self.pixel_to_meters(x_offset, y_offset, self.vehicle.location.global_relative_frame.alt)

	# 		# Adjust drone position towards marker - this function already implements proportional control
	# 		self.adjust_drone_position(x_offset_meters, y_offset_meters)  # Negative y to adjust for NED coordinate system

	# 		# Check if drone is close enough to the marker for landing
	# 		if abs(x_offset_meters) < 0.5 and abs(y_offset_meters) < 0.1:  # Thresholds for "close enough"
	# 			self.get_logger().info("Drone is positioned above the marker. Initiating landing.")
	# 			break
	# 		time.sleep(0.01)  # Adjust frequency of adjustments here

	# 	self.get_logger().info("Landing sequence initiated.")



	
	def precision_unmoving_landing(self):
		"""
		Perform a precision landing on an unmoving target by first moving above the target.

		:param target_relative_position: The relative position of the target from the drone's current position.
		"""
		while self.targeted_aruco_marker_relative_coordinates is None:
			print('Searching for target...')
			time.sleep(1.0)

		# Calculate the global position of the marker
		current_altitude = self.vehicle.location.global_relative_frame.alt  # Maintain current altitude

		# Switch to GUIDED mode for controlled movement
		self.vehicle.mode = VehicleMode("GUIDED")
		while self.vehicle.mode != "GUIDED":
			print('Setting vehicle to GUIDED mode...')
			time.sleep(0.5)

		# Command to go to the position of the marker with a specified velocity
		# Assuming self.targeted_aruco_marker_relative_coordinates contains [x, y, z] relative positions in meters
		x, y = self.targeted_aruco_marker_relative_coordinates

		# Prepare a message to move the drone to the position above the marker
		# Here, z is negative because in NED, down is negative.
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target_system, target_component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
			0b0000111111000111,  # type_mask (only positions enabled)
			x, y, current_altitude,  # x, y, z positions in meters (relative to the drone)
			0, 0, 0,  # x, y, z velocity in m/s (not used here, assuming stationary target)
			0, 0, 0,  # x, y, z acceleration (not supported)
			0, 0)    # yaw, yaw_rate (not used)

		# Send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

		print('Moving to target...')

		 # Wait until the drone is above the center
		target_position_reached = False
		while not target_position_reached:
			# Assuming function to check if the current position is close to target coordinates
			if np.linalg.norm(self.targeted_aruco_marker_relative_coordinates) < 0.1:
				target_position_reached = True
				print("Target position reached.")
			else:
				print("Adjusting position...")
				time.sleep(1)  # Check every 1 second







	

	# def precision_unmoving_landing(self):
	# 	try:
	# 		print('dddddddddddddd')
	# 		while self.targeted_aruco_marker_relative_coordinates is None:
	# 			print('eeeeeeeeeeeeeeeeeeeee')
	# 			time.sleep(0.1)
	# 		if self.targeted_aruco_marker_relative_coordinates is not None:	
	# 			print('ffffffffffffffff')
	# 			# Convert pixel offsets to metric offsets using known scale factors
	# 			# Scale factors depend on altitude and camera calibration
	# 			# This is a placeholder for actual conversion based on your system's specifics
	# 			x_offset_meters, y_offset_meters = self.pixel_to_meters(
	# 				self.targeted_aruco_marker_relative_coordinates[0],
	# 				self.targeted_aruco_marker_relative_coordinates[1],
	# 				self.vehicle.location.global_relative_frame.alt
	# 			)
				
	# 			self.adjust_drone_position(x_offset_meters, y_offset_meters)

	# 			# if abs(x_offset_meters) <= 0.1 and abs(y_offset_meters) <= 0.1:
	# 			# 	print('cccccccccccc')
	# 			# 	# The drone is approximately above the marker, initiate landing
	# 			# 	self.get_logger().info('Drone is above the marker, initiating landing.')
	# 			# 	self.vehicle.mode = VehicleMode('LAND')

						
	# 	except Exception as e:
	# 		self.get_logger().error(f"Error in precision landing: {e}")

	def adjust_drone_position(self, x_offset, y_offset):
		"""
		Adjusts the drone's position to center it above the ArUco marker using proportional control.

		:param x_offset: The horizontal offset (in meters) from the drone to the marker along the North axis.
		:param y_offset: The lateral offset (in meters) from the drone to the marker along the East axis.
		"""
		print(f"Adjusting position with x_offset: {x_offset}, y_offset: {y_offset}")

		# Proportional gain factors (tune these based on testing for optimal performance)
		kp_x = 0.2
		kp_y = 0.2

		# Ensure the drone is in GUIDED mode for movement control

		
		print('kkkkkkkkkkkkkkkkkkkkk')

		# Calculate velocity commands based on offset and proportional gain
		# The velocity decreases as the drone gets closer to the target
		velocity_x = kp_x * x_offset
		velocity_y = kp_y * y_offset
		velocity_z = 0  # Assuming no altitude change for now

		# Limit the maximum velocity to prevent too aggressive movements
		max_velocity = 15.0  # meters/sec, adjust as needed
		velocity_x = max(min(velocity_x, max_velocity), -max_velocity)
		velocity_y = max(min(velocity_y, max_velocity), -max_velocity)

		print(f"Sending velocity command: vx={velocity_x}, vy={velocity_y}, vz={velocity_z}")
		self.send_ned_velocity(velocity_x, velocity_y, velocity_z)

	def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
		"""
		Move vehicle in direction based on specified velocity vectors.

		:param velocity_x: Velocity in the North direction in meters/second.
		:param velocity_y: Velocity in the East direction in meters/second.
		:param velocity_z: Velocity in the Down direction in meters/second.
		"""
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target_system, target_component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
			0b0000111111000111,  # type_mask (only positions enabled)
			0, 0, 0,  # x, y, z positions (not used)
			velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
			0, 0, 0,  # x, y, z acceleration (not supported)
			0, 0)    # yaw, yaw_rate (not used)

		# Send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

	def pixel_to_meters(self, x_offset, y_offset, altitude):
		"""
		Converts pixel offsets to meters based on drone altitude and camera calibration.
		Placeholder for actual conversion logic.
		"""
		# Placeholder conversion factors; adjust based on your camera and typical altitudes
		scale_x = altitude * 0.001
		scale_y = altitude * 0.001
		
		return x_offset * scale_x, y_offset * scale_y


	##########
	def handle_marker_detection(self, msg):
		"""
		Handle marker detection messages from the camera node.
		Adds the marker to the seen marker dictionary with its actualized position.
		If the targeted marker is seen, sets the corresponding boolean to true to allow the drone to land on it.
		"""

		try:
			marker_data = json.loads(msg.data)
			# print(f"Parsed marker data: {marker_data}")
			if marker_data is not None:
				for marker in marker_data:
					marker_id = marker['id']
					marker_position = marker['position']
					# Update the seen markers dictionary
					self.tag_positions[marker_id] = marker_position
					
					# Check if the seen marker is the targeted marker
					if marker_id == self.targeted_aruco_marker:
						# self.get_logger().info(f"Targeted marker {self.targeted_aruco_marker} seen at position: {marker_position}")
						self.target_marker_found.set()
						# self.target_marker_found = True
						# Here, you might want to initiate a landing sequence or adjust the drone's position
						# For simplicity, we're just setting the flag here
						# You could call self.adjust_for_landing() or a similar method to handle precision landing
		except Exception as e:
			self.get_logger().error(f"Error processing marker detection message: {e}")
	
	def handle_mission_request(self, request, response):
		self.get_logger().info('Starting mission2_land_on_unmoving_target')
		# Call the method to start the mission
		self.mission2_land_on_unmoving_target_async()
		# self.mission2_land_on_unmoving_target()
		# Set the response
		response.success = True
		response.message = "Mission started"
		return response


	def command_callback(self, msg):
		command = msg.data
		self.get_logger().info(f'Received command: {command}')
		# Parse and execute commands
		if command.startswith("configure_vehicle"):
			self.configure_vehicle()
		elif command.startswith("takeoff"):
			altitude = float(command.split("; altitude: ")[1])
			self.get_logger().info(f'Send command to take off at: {altitude}')
			self.arm_and_takeoff(altitude)
		elif command.startswith("setup_grid_search"):
			bounds_str = command.split("; bounds: ")[1]
			bounds = self.parse_bounds(bounds_str)
			self.waypoints = self.generate_grid_search_waypoints(bounds)
		elif command.startswith("start_grid_search"):
			self.start_grid_search()
		elif command.startswith("land_on_marker"):
			pass
		elif command.startswith("initiate_landing"):
			self.initiate_landing()
	
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

		# msg = String()
		# msg.data = "has taken off"

		# self.drone_status_pub.publish(msg)

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

		print("Starting grid search...")
		while not self.target_marker_found.is_set():
			# Keep searching...
			pass

		# Once the target marker is found, clear the waypoints and prepare for landing
		if self.target_marker_found.is_set():
			self.vehicle.commands.clear()
			self.vehicle.commands.upload()
			print("Target marker found, preparing for precision landing...")
			# Transition to precision landing behavior
			self.search_ongoing = False
			self.precision_unmoving_landing_enabled = True

			newlat, newlon = self.get_global_position_of_marker()

			print('aaaaaaaaaaa', newlat, newlon)

			cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, newlat, newlon, self.takeoff_height)
			cmds.add(cmd)
			self.vehicle.commands.upload()

			self.vehicle.mode = VehicleMode("AUTO")
			while self.vehicle.mode != "AUTO":
				time.sleep(0.2)

			while np.linalg.norm(self.targeted_aruco_marker_relative_coordinates) > 0.1:
				time.sleep(0.1)
				print(self.targeted_aruco_marker_relative_coordinates)

			self.initiate_landing()
	
	

	def goto(self, targetLocation):
		
		# input is a LocationGlobalRelative variable
		distanceToTargetLocation = get_distance_meters(targetLocation, self.vehicle.location.global_relative_frame)

		self.vehicle.simple_goto(targetLocation,50)

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



	# send a velocity command with a +x being the heading of the drone 
	def send_local_ned_velocity(self, vx, vy, vz):
		print('drone goes brrrrrrrrrrrrrrr')
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,
			0,0,
			mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
			0b0000111111000111,  # .. BITMASK -> consider only the velocities 
			0, 0, 0,  # position
			vx, vy, vz,  # velocity 
			0, 0, 0,  # acceleration
			0, 0)
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush() 


	# send a velocity command with +x being the true north of earth 
	def send_global_ned_velocity(self, vx, vy, vz):
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,  # time_boot_ms (not used)
			0,0,  # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame  
			0b0000111111000111,  # .. BITMASK -> consider only the velocities 
			0, 0, 0,  # position
			vx, vy, vz,  # velocity 
			0, 0, 0,  # acceleration
			0, 0)
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()

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

	def send_land_message(self, x, y): 
		# for precision landing
		msg = self.vehicle.message_factory.landing_target_encode(
				0,
				0,
				mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
				x,
				y,
				0,0,0
			)
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


	# 	# parser.add_argument('--connect')
	# 	# args = parser.parse_args()

	# 	# connection_string = args.connect 

	# 	# if not connection_string:
	# 	# 	import dronekit_sitl 
	# 	# 	sitl = dronekit_sitl.start_default()simple_takeoff
	# 	# 	connection_string = sitl.connection_string() 
			
	# 	# self.vehicle = connect(connection_string, wait_ready=True)
	# 	vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

	# 	return vehicle 


	# send a velocity command with a +x being the heading of the drone 


	# send a velocity command with +x being the true north of earth 
	# fly to waypoint. we use ti as a dummy function to setup the condition_yaw function 
	
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
		
	
	############ OpenCV scripts ##############
		
	def decode_predictions(self, scores, geometry):
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

	def generate_grid_search_waypoints(self, bounds=None):
		"""
		Generate a grid of waypoints to cover the search area.
		
		:param bounds: A dictionary with 'north', 'south', 'east', 'west' as keys and their respective coordinate values.
		:param waypoint_spacing: The spacing between waypoints in meters.
		:return: A list of waypoints (latitude, longitude).
		"""


		if bounds == None:
			bounds = self.bounds
		
		north_gps = get_gps_from_distance([self.lat_home, self.lon_home], 0, np.abs(bounds['north']) / 1000.0)
		south_gps = get_gps_from_distance([self.lat_home, self.lon_home], math.pi, np.abs(bounds['south']) / 1000.0)
		east_gps = get_gps_from_distance([self.lat_home, self.lon_home], math.pi / 2, np.abs(bounds['east']) / 1000.0)
		west_gps = get_gps_from_distance([self.lat_home, self.lon_home], 3 * math.pi / 2, np.abs(bounds['west']) / 1000.0)

		bounds_gps = {
			'north': north_gps[0],
			'south': south_gps[0],
			'east': east_gps[1],
			'west': west_gps[1]
		}

		# Convert waypoint spacing from meters to degrees (approximation)
		spacing_in_degrees = self.waypoint_spacing / 111111  # Roughly 111,111 meters in a degree

		# Generate grid points
		lat_points = np.arange(bounds_gps['south'], bounds_gps['north'], spacing_in_degrees)
		lon_points = np.arange(bounds_gps['west'], bounds_gps['east'], spacing_in_degrees)

		waypoints = []
		for i, lat in enumerate(lat_points):
			# Convert lon_points to a list for the index method
			lon_list = lon_points.tolist()
			for lon in lon_list:
				if i % 2 == 0:
					waypoints.append((lat, lon))
					print(lat, lon)
				else:
					# Use the reversed index for the back-and-forth pattern
					waypoints.append((lat, lon_list[-(lon_list.index(lon) + 1)]))


		return waypoints






def main():
	rclpy.init() #init routine needed for ROS2.

	vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
	drone = Drone(vehicle) #Create class object to be used.

	executor = rclpy.executors.MultiThreadedExecutor()
	executor.add_node(drone)
	try:
		executor.spin()
	finally:
		drone.destroy_node()
		rclpy.shutdown()



if __name__ == '__main__':
	main()
    