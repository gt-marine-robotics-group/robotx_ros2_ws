import cv2
import cv2.aruco as aruco
import numpy as np

def create_recursive_aruco_markers():
    # Check OpenCV version
    print("OpenCV Version: ", cv2.__version__)

    # Define marker IDs and sizes
    outer_marker_id = 1
    inner_marker_id = 72
    outer_size = 100  # Size in pixels
    inner_size = 100  # Size in pixels

    inner_bg_size = inner_size + 10

    background_size = 160

    # Get the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    
    # Create outer marker
    outer_marker = aruco.generateImageMarker(aruco_dict, outer_marker_id, outer_size)
    # Create inner marker
    inner_marker = aruco.generateImageMarker(aruco_dict, inner_marker_id, inner_size)

    # Create a gray background for the inner marker


    return outer_marker, inner_marker


def detect_and_annotate_markers(image, arucoDetector):
    # Convert to grayscale if needed
    if len(image.shape) == 2 or image.shape[2] == 1:
        gray = image
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = arucoDetector.detectMarkers(gray)


    if ids is not None:
        # aruco.drawDetectedMarkers(image, corners, ids)

        for id, corner in zip(ids, corners):
            # Calculate the center position of the marker
            center_x = int(corner[0][0][0] + corner[0][2][0]) // 2
            center_y = int(corner[0][0][1] + corner[0][2][1]) // 2

            # Get the marker ID as a string
            marker_id_str = str(id[0])

            # Choose font scale and thickness according to your image size
            font_scale = 0.7
            thickness = 1

            # Get the text size for the ID to position it properly
            #text_size = cv2.getTextSize(marker_id_str, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]

            # Calculate the bottom-left corner of the text
            # text_x = center_x - text_size[0] // 2
            # text_y = center_y + text_size[1] // 2

            # Put the text on the image
            #cv2.putText(image, marker_id_str, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness)
    else:
        print('No markers detected')

    return image, ids


# Generate the recursive marker
outer_marker, inner_marker = create_recursive_aruco_markers()

# Attempt to detect markers in the generated image



# Optionally, save the image to a file
cv2.imwrite("aruco_1.png", outer_marker)
cv2.imwrite("aruco_72.png", inner_marker)