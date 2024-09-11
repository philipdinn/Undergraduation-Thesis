import cv2
import numpy as np

# Load the image
frame = cv2.imread('TestData/Test9.jpg')

# Convert to HSV
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

# Define HSV ranges for orange, black, and white
low_orange = np.array([2, 100, 210])
high_orange = np.array([20, 210, 255])

low_black = np.array([0, 0, 0])
high_black = np.array([255, 80, 70])

low_white = np.array([0, 0, 200])
high_white = np.array([180, 30, 255])

# Create masks for each color
orange_mask = cv2.inRange(hsv_frame, low_orange, high_orange)
black_mask = cv2.inRange(hsv_frame, low_black, high_black)
white_mask = cv2.inRange(hsv_frame, low_white, high_white)

# Combine masks
combined_mask = cv2.bitwise_or(orange_mask, white_mask)
combined_mask = cv2.bitwise_or(combined_mask, black_mask)

# Find contours in each mask
contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours_black, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours_white, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Function to detect and label shapes based on contours
def detect_and_label_shapes(contours, frame, label, color):
    detected_shapes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 50000:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 10)
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            x, y, w, h = cv2.boundingRect(contour)  # Calculate bounding rectangle
            aspect_ratio = float(w) / h
            # print(len(approx))
            # print(area)
            if len(approx) == 3:
                detected_shapes.append((f"{label} Triangle"))
                cv2.putText(frame, f"{label} Triangle" ,(x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 3, color, 8)
            elif len(approx) == 4:
                detected_shapes.append((f"{label} Rectangle"))
                cv2.putText(frame, f"{label} Rectangle",(x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 3, color, 8)
            else:
                detected_shapes.append((f"{label} Circle"))
                cv2.putText(frame, f"{label} Circle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 3, color,8)
    return detected_shapes
            # if len(approx) == 3:
            #     cv2.putText(frame, f"{label} Triangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            # elif len(approx) == 4 and 0.5 <= aspect_ratio <= 1.5:
            #     cv2.putText(frame, f"{label} Rectangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            # elif len(approx) > 3 and area / (perimeter * perimeter) > 0.08:
            #     cv2.putText(frame, f"{label} Circle",(x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# Detect and label shapes for each color
detected_orange_shapes = detect_and_label_shapes(contours_orange, frame, "Orange", (0, 165, 255))  # Orange color
detected_black_shapes = detect_and_label_shapes(contours_black, frame, "Black", (0, 0, 0))       # Black color
detected_white_shapes = detect_and_label_shapes(contours_white, frame, "White", (255, 255, 255))  # White color
all_detected_shapes = detected_orange_shapes + detected_black_shapes + detected_white_shapes
print(all_detected_shapes)
# Resize the frame to a height of 640 while maintaining the aspect ratio
height, width = frame.shape[:2]
new_height = 480
new_width = int((new_height / height) * width)
resized_frame = cv2.resize(frame, (new_width, new_height))
resized_mask = cv2.resize(combined_mask, (new_width, new_height))
combined_display = np.hstack((resized_frame, cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)))
# Display the combined image
cv2.imshow("Detection Frame", resized_frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
