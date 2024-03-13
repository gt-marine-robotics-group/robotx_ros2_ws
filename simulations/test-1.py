#!/usr/bin/env python3

import subprocess
import time
import signal
import sys

# Configuration
ardupilot_path = "/home/hugues/robotx_ws/ardupilot"
robotx_gazebo_path = "/home/hugues/robotx_ws/robotx_gazebo/"
setup_path = "/home/hugues/robotx_ws/dev_ws/install/setup.bash"
ros2_package_name = "robotx"
ros2_node_executable = "search_and_report"  # The executable name of your ROS2 node

# Start Gazebo
def start_gazebo():
    print(f"Launching Gazebo with {robotx_gazebo_path}...")
    cmd = [
        "ros2",
        "launch",
        "gazebo_ros",
        "gazebo.launch.py",
        "world:=2022_qualifying_task_aruco.world"
    ]
    return subprocess.Popen(cmd, cwd=robotx_gazebo_path)

# Start ArduPilot SITL
def start_ardupilot():
    print("Starting ArduPilot SITL...")
    cmd = [
        "sim_vehicle.py",
        "-v", "ArduCopter",
        "-f", "gazebo-iris",
    ]
    return subprocess.Popen(cmd, cwd=ardupilot_path)

def sourcing_robotx():
    print('Sourcing robotx setup')
    cmd = [
        "source",
        setup_path
    ]
    return subprocess.Popen(cmd)

# Launch ROS2 nodes
def launch_ros2_nodes():
    print("Launching ROS2 nodes...")
    cmd = [
        "ros2", "run",
        ros2_package_name,
        f"{ros2_node_executable}"
    ]
    return subprocess.Popen(cmd)

def signal_handler(sig, frame):
    print('Shutting down...')
    global gazebo_process, ardupilot_process, ros2_process
    if gazebo_process: gazebo_process.terminate()
    if ardupilot_process: ardupilot_process.terminate()
    if ros2_process: ros2_process.terminate()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize processes
    gazebo_process = None
    ardupilot_process = None
    ros2_process = None

    try:
        # Source robotx
        # sorucing_process = sourcing_robotx()
        # time.sleep(1)

        # Start Gazebo
        gazebo_process = start_gazebo()
        time.sleep(10)  # Adjust sleep time based on your system's startup time

        # Start ArduPilot
        ardupilot_process = start_ardupilot()
        time.sleep(15)  # Adjust sleep time based on your system's startup time


        # Launch ROS2 nodes
        ros2_process = launch_ros2_nodes()

        # Keep the script running until Ctrl+C is pressed
        while True:
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        signal_handler(None, None)