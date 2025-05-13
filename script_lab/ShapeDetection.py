import cv2
import numpy as np

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        shape = 'unidentified'
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        min_x, max_x, min_y, max_y = 10000, 0, 10000, 0
        for point in c:
            x, y = point[0][0], point[0][1]
            if x > max_x: max_x = x
            if x < min_x: min_x = x
            if y > max_y: max_y = y
            if y < min_y: min_y = y
        dX, dY = max_x - min_x, max_y - min_y
        ratio = dX / (dY + 0.00001)

        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if 0.95 <= ar <= 1.05 else "rectangle"
        elif len(approx) == 5:
            shape = "pentagon"
        elif len(approx) >= 5 and 0.9 <= ratio <= 1.1:
            shape = "circle"
        else:
            shape = 'polygon'

        return shape, approx  # Return the approximated contour for drawing

if __name__ == "__main__":
    image_path = "aruco_image_1.png"
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    SD = ShapeDetector()

    for c in contours:
        shape, approx = SD.detect(c)
        cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))
        else:
            cX, cY = 0, 0
        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Show the image
    cv2.imshow("Image with Detected Shapes", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
