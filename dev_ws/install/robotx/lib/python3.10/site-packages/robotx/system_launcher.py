import rclpy
from rclpy.executors import MultiThreadedExecutor
# Import your node classes
from drone_node import DroneNode
from camera_node import CameraNode
from mission_node import MissionCommandNode
from dronekit import connect

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 communication

    # Create instances of your nodes
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)  # Assuming all nodes use the same vehicle connection
    drone_node = DroneNode(vehicle)
    camera_node = CameraNode() 
    mission_command_node = MissionCommandNode()

    executor = MultiThreadedExecutor(num_threads=6)  # Allows nodes to run concurrently

    # Add your nodes to the executor
    executor.add_node(drone_node)
    executor.add_node(camera_node)
    executor.add_node(mission_command_node)

    try:
        # Spin the executor to process callbacks from all nodes
        executor.spin()
    finally:
        # Clean up resources and shutdown nodes
        drone_node.destroy_node()
        camera_node.destroy_node()
        mission_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
