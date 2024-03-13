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
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
import json
import threading

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


class MissionCommandNode(Node):
    def __init__(self):
        super().__init__('mission_command')  # This initializes the ROS 2 node

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
        self.drone_command_pub = self.create_publisher(String, '/drone_command', 10)
        self.waypoint_pub = self.create_publisher(PoseArray, '/drone/waypoint', 10)
        self.target_marker_position_pub = self.create_publisher(Point, '/drone/target_marker_position', 10)  # Same assumption

        qos_profile = QoSProfile(depth=10)
        # self.video_sub = self.create_subscription(Image, '/camera/image_raw', self.video_feed_processing, qos_profile)
        self.mission_service = self.create_service(Trigger, 'start_mission2', self.handle_mission_request)

        self.camera_sub = self.create_subscription(String, '/drone/marker_detection', self.callback_marker, 10)
        self.mission_state_subscriber = self.create_subscription(String, '/drone_status', self.mission_state_callback, 10)

        self.home_position_subscription = self.create_subscription(
            String,
            '/home_position',
            self.home_position_callback,
            10)


        # self._action_server = ActionServer(
        # 	self,
        # 	DroneOperation,
        # 	'drone_command',
        # 	self.execute_callback)

        # qos_camera = QoSProfile(depth=10)
        # self.camera_pub  = self.create_publisher(Image, '/camera/color/image_raw', qos_camera)

        self.has_taken_off = False
        self.search_ongoing = False
        self.landing_allowed = False
        self.marker_found = False
        self.last_waypoint_reached = False
        # self.target_marker_found = False
        self.target_marker_found = threading.Event()
        self.bounds = {'north': 20, 'south': -20, 'east': 15, 'west': -15}
        self.tag_positions = {}
        self.currently_seen_markers = []
        self.targeted_aruco_marker = 72
        self.waypoint_spacing = 5
        self.targeted_aruco_marker_relative_coordinates = None

        self.one_shot_timer = self.create_timer(25.0, self.one_shot_callback)



    ## Start and do the mission
        
    def home_position_callback(self, msg):
        home_position = json.loads(msg.data)
        self.lat_home = home_position['lat']
        self.lon_home = home_position['lon']
        
    def send_command(self, cmd):
        """
        Send commands from this node to drone node
        """
        msg = String()
        msg.data = cmd 
        self.drone_command_pub.publish(msg)

    def one_shot_callback(self):
        self.mission2_land_on_unmoving_target_async()
        self.one_shot_timer.cancel()  # Stop the timer after the first executio

    def mission2_land_on_unmoving_target_async(self):
        self.one_shot_timer.cancel()
        # Wrap the original function to run in a separate thread
        thread = threading.Thread(target=self.mission2_land_on_unmoving_target)
        thread.start()

    def mission2_land_on_unmoving_target(self):
        # Wait for the node to fully initialize
        time.sleep(2)

        # self.get_logger().info("Configuring vehicle parameters...")
        # self.send_command("configure_vehicle")

        self.get_logger().info("Taking off...")
        self.send_command("takeoff; altitude: 10.0")

        while not self.has_taken_off:
            time.sleep(1.0)

        self.search_ongoing = True

        self.waypoints = self.generate_grid_search_waypoints()
        self.publish_waypoints(self.waypoints)

        time.sleep(1.0)

        
        self.send_command("start_grid_search")

        while self.search_ongoing and not self.marker_found:
            time.sleep(0.1)

        if self.marker_found:
            self.send_command("clear_command_line")
            self.send_command("land_on_marker")
        else:
            self.send_command("initiate_landing")

        # Assume some mechanism or monitoring to determine when to land
        # This could be based on time, sensor data, or manual intervention
        # For simplicity, we'll just wait a fixed amount of time
        time.sleep(30)  # Placeholder for actual search time
        self.get_logger().info(f'End of allllllllllllllll')

        # self.get_logger().info("Grid search complete, preparing to land...")
        # self.send_command("initiate_landing")

    def handle_mission_request(self, request, response):
        self.get_logger().info('Starting mission2_land_on_unmoving_target')
        # Call the method to start the mission
        self.mission2_land_on_unmoving_target_async()
        # self.mission2_land_on_unmoving_target()
        # Set the response
        response.success = True
        response.message = "Mission started"
        return response

    def mission_state_callback(self, msg):
        if msg.data == "has taken off":
            self.has_taken_off = True
        elif msg.data == "search is over":
            self.search_ongoing = False
        elif msg.data == "marker found":
            self.marker_found = True


    def callback_marker(self, msg):
        if self.search_ongoing and not self.marker_found:
            self.handle_marker_detection(msg)
        elif self.marker_found:
            marker_data = json.loads(msg.data)
            for marker in marker_data:
                if marker['id'] == self.targeted_aruco_marker:
                    # Calculate the position offsets from the drone to the marker
                    x_offset = float(marker['position']['x']) - (self.horizontal_res / 2)  # Assuming center of image as origin
                    y_offset = float(marker['position']['y']) - (self.vertical_res / 2)  # Assuming center of image as origin
                    self.targeted_aruco_marker_relative_coordinates = np.array([x_offset, y_offset])
                    self.publish_target_marker_position(self.targeted_aruco_marker_relative_coordinates)


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
                        self.get_logger().info(f"Targeted marker {self.targeted_aruco_marker} seen at position: {marker_position}")
                        self.marker_found = True
        except Exception as e:
            self.get_logger().error(f"Error processing marker detection message: {e}")

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

    def publish_waypoints(self, waypoints):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "map"  # or whatever the relevant frame of reference is

        for wp in waypoints:
            pose = Pose()
            pose.position.x = wp[0]  # Latitude
            pose.position.y = wp[1]  # Longitude
            # Assuming altitude or other data isn't critical for your waypoints
            # If needed, consider using position.z for altitude
            pose_array_msg.poses.append(pose)

        self.waypoint_pub.publish(pose_array_msg)


    def publish_target_marker_position(self, position):
        if position is None or not isinstance(position, (list, np.ndarray)) or len(position) < 2:
            self.get_logger().error('Invalid position data')
            return
        marker_position_msg = Point()
        marker_position_msg.x = position[0]  # Assuming the first element is x
        marker_position_msg.y = position[1]  # Assuming the second element is y
        marker_position_msg.z = 0.0  # Assuming the marker is on the ground or relative height is not considered
        self.target_marker_position_pub.publish(marker_position_msg)



# def main():
#     rclpy.init() #init routine needed for ROS2.
#     mission_command = MissionCommandNode() #Create class object to be used.

#     executor = rclpy.executors.MultiThreadedExecutor()
#     executor.add_node(mission_command)
#     try:
#         executor.spin()
#     finally:
#         mission_command.destroy_node()
#         rclpy.shutdown()
        

def main():
	rclpy.init() #init routine needed for ROS2.
	mission = MissionCommandNode() #Create class object to be used.


	rclpy.spin(mission)

	mission.destroy_node()

	rclpy.shutdown()



if __name__ == '__main__':
	main()