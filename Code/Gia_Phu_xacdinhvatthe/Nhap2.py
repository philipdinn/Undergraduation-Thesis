import cv2
import numpy as np

# Function to detect and label shapes based on contours
def detect_and_label_shapes(contours, frame, label, color):
    detected_shapes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 15000 and area < 35000:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            perimeter = cv2.arcLength(contour, True)
            print(area)
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            x, y, w, h = cv2.boundingRect(contour)  # Calculate bounding rectangle
            aspect_ratio = float(w) / h
            if len(approx) == 3:
                detected_shapes.append(f"{label} Triangle")
                cv2.putText(frame, f"{label} Triangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
            elif len(approx) == 4 and 0.5 <= aspect_ratio <= 1.5:
                detected_shapes.append(f"{label} Rectangle")
                cv2.putText(frame, f"{label} Rectangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
            elif len(approx) > 6 and area / (perimeter * perimeter) > 0.05:
                detected_shapes.append(f"{label} Circle")
                cv2.putText(frame, f"{label} Circle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
    return detected_shapes

username = 'giaphu'
password = '123456'
ip_address = '192.168.1.4'
port = '8080'
# URL với thông tin đăng nhập
ip_webcam_url = f'http://{username}:{password}@{ip_address}:{port}/video'

# Open webcam
cap = cv2.VideoCapture(ip_webcam_url)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break

    # Convert to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    low_orange = np.array([155, 100, 180])
    high_orange = np.array([178, 200, 255])
    low_blue = np.array([20, 128, 180])
    high_blue = np.array([50, 240, 255])
    low_white = np.array([0, 0, 150])
    high_white = np.array([180, 100, 255])

    orange_mask = cv2.inRange(hsv_frame, low_orange, high_orange)
    blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    white_mask = cv2.inRange(hsv_frame, low_white, high_white)

    combined_mask = cv2.bitwise_or(orange_mask, white_mask)
    combined_mask = cv2.bitwise_or(combined_mask, blue_mask)

    contours_orange, _ = cv2.findContours(orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_white, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    detected_orange_shapes = detect_and_label_shapes(contours_orange, frame, "Pink", (119, 55, 230))
    detected_blue_shapes = detect_and_label_shapes(contours_blue, frame, "Yellow", (55, 249, 249))
    detected_white_shapes = detect_and_label_shapes(contours_white, frame, "White", (255, 255, 255))
    all_detected_shapes = detected_orange_shapes + detected_blue_shapes + detected_white_shapes
   
    # Print detected shapes to the terminal
    if all_detected_shapes:
        print("Detected shapes:", all_detected_shapes)
    # Resize the frame to a height of 480 while maintaining the aspect ratio
    #Hien thi len tren cua so windows
    height, width = frame.shape[:2]
    new_height = 480
    new_width = int((new_height / height) * width)
    resized_frame = cv2.resize(frame, (new_width, new_height))
    resized_mask = cv2.resize(combined_mask, (new_width, new_height))
    resized_HSV_frame = cv2.resize(hsv_frame, (new_width, new_height))

    # Combine frame and mask for display
    combined_display = np.hstack((resized_frame, cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)))
    # Display the combined image
    #cv2.imshow("Detection Frame", combined_display)
    cv2.imshow("Frame", frame)                   # Hiển thị khung hình "Frame" đã thay đổi kích thước
    cv2.imshow("Detection Frame", resized_mask)
    cv2.imshow("HSV_Frame", resized_HSV_frame)
    
    print(frame.shape)
    print(frame.size)
    print(frame.dtype)
    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()
