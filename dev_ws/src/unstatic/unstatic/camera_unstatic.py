import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
import cv2.aruco as aruco
import numpy as np
import json
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            String,
            '/drone/marker_detection',
            10)
        
        self.publisher

        self.special_publisher = self.create_publisher(
            String,
            '/drone/marker_35_detection',
            10)
        
        self.debug_publisher = self.create_publisher(
            Image,
            '/drone/annoted_image',
            10)
        
        self.debug_publisher
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.arucoDetector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.convert_ros_image_to_opencv(msg)
        
        # Process the image to detect ArUco markers
        marker_info, annotated_frame = self.detect_aruco_markers(frame)
        
        # Iterate over detected markers and publish based on specific IDs
        for marker in marker_info:
            if marker['id'] == 1 or marker['id'] == 72:
                # Publish marker 1 info to the general topic
                self.publisher.publish(String(data=json.dumps(marker)))
            elif marker['id'] == 35:
                # Publish marker 35 info to the special topic
                self.special_publisher.publish(String(data=json.dumps(marker)))
        
        # Convert the annotated OpenCV image back to ROS and publish it
        try:
            # Convert the annotated OpenCV image back to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            # Publish the ROS Image message
            self.debug_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to convert and publish annotated image: {e}')






    def convert_ros_image_to_opencv(self, data):
        try:
            frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS Image to OpenCV: {e}')
            return None

    def detect_aruco_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.arucoDetector.detectMarkers(gray)

        marker_info = []

        if ids is not None:
            for i, corner_group in zip(ids, corners):
                # Extracting the corner points
                corner = corner_group[0]
                # Converting to integer values
                int_corners = np.int0(corner)
                
                # Drawing the polygon around the marker
                cv2.polylines(frame, [int_corners], isClosed=True, color=(0, 255, 0), thickness=3)
                
                # Calculating the center of the marker for placing text
                centerX = int(sum([c[0] for c in corner]) / 4)
                centerY = int(sum([c[1] for c in corner]) / 4)
                
                # Adding marker information for publishing
                marker_info.append({"id": int(i[0]), "position": {"x": centerX, "y": centerY}})
                
                # Placing the ID text at the center of the marker
                cv2.putText(frame, str(i[0]), (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Calculate the exact center of the image
        center_of_image = (frame.shape[1] // 2, frame.shape[0] // 2)

        # Draw a small cross at the center of the image
        cv2.line(frame, (center_of_image[0] - 10, center_of_image[1]), (center_of_image[0] + 10, center_of_image[1]), (255, 0, 0), 2)
        cv2.line(frame, (center_of_image[0], center_of_image[1] - 10), (center_of_image[0], center_of_image[1] + 10), (255, 0, 0), 2)

        return marker_info, frame
    

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
