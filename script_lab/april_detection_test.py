import cv2
import apriltag
import numpy as np
import matplotlib.pyplot as plt

def detect_apriltags(image_path):
    # Load the image
    image = cv2.imread(image_path)
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Create an AprilTag detector object
    detector = apriltag.Detector()
    
    # Detect the AprilTags in the image
    tags = detector.detect(gray_image)
    
    # Check if any AprilTags were detected
    if tags:
        # Iterate over each detected tag
        for tag in tags:
            # Extract the corners of the tag
            corners = np.array(tag.corners).astype(int)
            
            # Draw the bounding box
            for i in range(4):
                cv2.line(image, tuple(corners[i]), tuple(corners[(i+1)%4]), (0, 255, 0), 2)
            
            # Calculate the center of the tag
            center = np.mean(corners, axis=0).astype(int)
            
            # Draw a small circle at the center
            cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
            
            # Draw the tag's ID on the image
            cv2.putText(image, str(tag.tag_id), tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
        # Display the image with AprilTags highlighted
        plt.figure(figsize=(8, 8))
        plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        plt.title("Detected AprilTags with Centers")
        plt.show()
    else:
        print("No AprilTags detected.")

if __name__ == "__main__":
    # Assuming the image is named "image.png" and located in the same directory as this script
    image_path = "april_test_2.png"
    detect_apriltags(image_path)
