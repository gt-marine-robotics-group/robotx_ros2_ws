import cv2
import cv2.aruco as aruco
import numpy as np

def create_recursive_aruco_markers():
    # Check OpenCV version
    print("OpenCV Version: ", cv2.__version__)

    # Define marker IDs and sizes
    outer_marker_id = 1
    inner_marker_id = 72
    outer_size = 140  # Size in pixels
    inner_size = 25  # Size in pixels

    inner_bg_size = inner_size + 10

    background_size = 160

    # Get the ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    
    # Create outer marker
    outer_marker = aruco.generateImageMarker(aruco_dict, outer_marker_id, outer_size)
    # Create inner marker
    inner_marker = aruco.generateImageMarker(aruco_dict, inner_marker_id, inner_size)

    # Create a gray background for the inner marker
    inner_background = np.full((inner_bg_size, inner_bg_size), 125, dtype=np.uint8)
    inner_background_with_marker = cv2.cvtColor(inner_background, cv2.COLOR_GRAY2BGR)
    inner_marker_colored = cv2.cvtColor(inner_marker, cv2.COLOR_GRAY2BGR)
    start_point = (inner_bg_size - inner_size) // 2
    inner_background_with_marker[start_point:start_point+inner_size, start_point:start_point+inner_size] = inner_marker_colored

    # Place the gray background with the inner marker onto the outer marker
    start_point = (outer_size - inner_bg_size) // 2
    outer_marker_colored = cv2.cvtColor(outer_marker, cv2.COLOR_GRAY2BGR)
    outer_marker_colored[start_point:start_point+inner_bg_size, start_point:start_point+inner_bg_size] = inner_background_with_marker

    # Create a gray background for the entire marker
    background = np.full((background_size, background_size), 125, dtype=np.uint8)
    background = cv2.cvtColor(background, cv2.COLOR_GRAY2BGR)

    # Place the outer marker onto the background
    start_point = (background_size - outer_size) // 2
    background[start_point:start_point + outer_size, start_point:start_point + outer_size] = outer_marker_colored

    


    # Calculate the region of interest (ROI) around the inner marker
    border_size = 5
    roi_start_point = start_point + (outer_size - inner_size) // 2 - border_size
    roi_end_point = roi_start_point + inner_size + (2 * border_size)
    inner_marker_roi_with_border = background[roi_start_point:roi_end_point, roi_start_point:roi_end_point]

    # Decide the size of the zoomed image
    zoom_size = 200  # For example, zoom to 200x200 pixels

    # Resize (zoom in) the ROI to the new size
    zoomed_in_marker = cv2.resize(inner_marker_roi_with_border, (zoom_size, zoom_size), interpolation=cv2.INTER_NEAREST)

    return background, zoomed_in_marker


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
background, zoomed_inner_marker = create_recursive_aruco_markers()

# Attempt to detect markers in the generated image

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
arucoDetector = aruco.ArucoDetector(aruco_dict, parameters)
annotated_image, ids_background = detect_and_annotate_markers(background.copy(), arucoDetector)
annotated_image, ids_zoom = detect_and_annotate_markers(zoomed_inner_marker.copy(), arucoDetector)

print(f'ids detected background: {ids_background}')
print(f'ids detected zoom: {ids_zoom}')
# Display the marker and the detection result
cv2.imshow("Background with Recursive ArUco Marker", background)
cv2.imshow("Zoomed Inner Marker", zoomed_inner_marker)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Optionally, save the image to a file
cv2.imwrite("aruco_custom.png", background)
cv2.imwrite("zoomed_inner_marker.png", zoomed_inner_marker)