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
    
    # Initialize the detector parameters using default values
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()  # Ensure correct method to create parameters
    
    # Detect the markers in the image
    corners, ids, rejected_candidates = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

    print(corners)
    
    if ids is not None:
        # Annotate detected markers
        annotated_image = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
        
        for corner in corners:
            # Calculate the precise center of the marker
            center = np.mean(corner[0], axis=0)
            center_int = tuple(np.round(center).astype(int))  # Convert to integer for drawing
            
            # Draw a small circle at the precise center
            cv2.circle(annotated_image, center_int, 5, (255, 0, 0), -1)
        
        # Display the image with markers and precise centers highlighted
        plt.figure(figsize=(8, 8))
        plt.imshow(cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB))
        plt.title("Detected ArUco Markers with Precise Centers")
        plt.show()
    else:
        print("No ArUco markers detected.")

if __name__ == "__main__":
    image_path = "aruco_image_1.png"
    detect_aruco_markers_and_centers(image_path)
