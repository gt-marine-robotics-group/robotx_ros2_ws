import cv2
import matplotlib.pyplot as plt
import numpy as np

def detect_aruco_markers_and_centers(image_path):
    # Load the image
    image = cv2.imread(image_path)
    
    if image is None:
        print(f"Failed to load image at {image_path}")
        return

    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # List of all predefined ArUco dictionaries in OpenCV
    aruco_dict_names = [
        cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_4X4_250, cv2.aruco.DICT_4X4_1000,
        cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_5X5_250, cv2.aruco.DICT_5X5_1000,
        cv2.aruco.DICT_6X6_50, cv2.aruco.DICT_6X6_100,
        cv2.aruco.DICT_6X6_250, cv2.aruco.DICT_6X6_1000,
        cv2.aruco.DICT_7X7_50, cv2.aruco.DICT_7X7_100,
        cv2.aruco.DICT_7X7_250, cv2.aruco.DICT_7X7_1000,
        cv2.aruco.DICT_ARUCO_ORIGINAL
    ]

    # Initialize the detector parameters using default values
    parameters = cv2.aruco.DetectorParameters()
    
    for aruco_dict_name in aruco_dict_names:
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
        
        # Detect the markers in the image
        corners, ids, rejected_candidates = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)
        
        if ids is not None:
            print(f"Markers found with dictionary: {aruco_dict_name}")
            # Annotate detected markers
            annotated_image = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
            
            # Iterate through each detected marker to compute and draw the center
            for corner in corners:
                # Calculate the center of the marker
                center = np.mean(corner[0], axis=0).astype(int)
                
                # Draw a small circle at the center
                cv2.circle(annotated_image, tuple(center), 5, (255, 0, 0), -1)
            
            # Display the image with markers and centers highlighted
            plt.figure(figsize=(8, 8))
            plt.imshow(cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB))
            plt.title(f"Detected ArUco Markers with Centers using Dictionary {aruco_dict_name}")
            plt.show()
            return  # Stop after finding the first dictionary that works
    else:
        print("No ArUco markers detected with any dictionary.")

if __name__ == "__main__":
    # Assuming the image is named "image.png" and located in the same directory as this script
    image_path = "aruco_image_1.png"
    detect_aruco_markers_and_centers(image_path)
