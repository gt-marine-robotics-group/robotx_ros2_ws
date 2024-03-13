#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time
from rclpy.action import ActionServer, ActionClient
# from robotx.action import DroneOperation  # Replace with your actual package name and action file name



class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.command_publisher = self.create_publisher(String, '/drone_command', 10)
        self.mission_state_subscriber = self.create_subscription(String, '/drone_status', self.mission_state_callback, 10)

        self.has_taken_off = False
        self.search_ongoing = False
        self.marker_found = False

        # self._action_client = ActionClient(self, DroneOperation, 'drone_command')
        self.initiate_mission()

    # def send_command(self, command):
    #     goal_msg = DroneOperation.Goal()
    #     goal_msg.command = command
        
    #     self._action_client.wait_for_server()
    #     self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    #     self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
    
    def mission_state_callback(self, msg):
        if msg.data == "has taken off":
            self.has_taken_off = True
        elif msg.data == "search is over":
            self.search_ongoing = False
        elif msg.data == "marker found":
            self.marker_found = True
        
    def wait_for_takeoff(self):
        while not self.has_taken_off:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def wait_for_end_of_search(self):
        while self.search_ongoing:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.command_publisher.publish(msg)


    def initiate_mission(self):
        # Wait for the node to fully initialize
        time.sleep(2)

        # self.get_logger().info("Configuring vehicle parameters...")
        # self.send_command("configure_vehicle")

        self.get_logger().info("Taking off...")
        self.send_command("takeoff; altitude: 10.0")

        self.wait_for_takeoff()

        self.search_ongoing = True

        self.get_logger().info("Setting up grid search waypoints...")
        self.send_command("setup_grid_search; bounds: north=20, south=-20, east=15, west=-15")

        time.sleep(1.0)

        self.get_logger().info("Starting grid search...")
        self.send_command("start_grid_search")

        self.wait_for_end_of_search()

        if self.marker_found:
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

def main(args=None):
    rclpy.init(args=args)
    mission_control = MissionControl()
    rclpy.spin(mission_control)

    mission_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
