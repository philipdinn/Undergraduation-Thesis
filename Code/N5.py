import cv2  # xử lý, nhận diện h/ả
import numpy as np  # mảng, ma trận, tính toán, xử lý h/ả
import time  # độ trễ, tg hiện tại, tg thực thi 
import json  # lưu trữ và truyền tải
import paho.mqtt.client as mqtt  # kết nối nối Broker của MQTT
import serial
import threading  # chạy các tác vụ đồng thời (xử lý hả và gửi dữ liệu qua mạng)
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # '/dev/ttyUSB0' có thể thay đổi tùy vào cổng serial bạn sử dụng
except:
    ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
time.sleep(2)

# Hàm callback khi kết nối MQTT thành công
def on_connect(client, userdata, flags, rc):  # rc: Mã kết quả của kết nối (result code).
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Connection successful")
        client.publish(topic, message)
    else:
        print("Connection failed")
        
def detect_and_label_shapes(contours, frame, label, color):
    detected_shapes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 10000:        # Kiểm tra xem diện tích của đường viền có lớn hơn 10000 hay không. Chỉ xử lý các đường viền có diện tích lớn hơn giá trị này để bỏ qua các đối tượng nhỏ không quan trọng.
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)      # -1 vẽ đường viền trong frame, xanh lá, độ dày 2 pixel
            perimeter = cv2.arcLength(contour, True)       # 
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            x, y, w, h = cv2.boundingRect(contour)  # Calculate bounding rectangle
            aspect_ratio = float(w) / h
            if len(approx) == 3:
                detected_shapes.append((label, "Triangle"))
                cv2.putText(frame, f"{label} Triangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
            elif len(approx) == 4:
                detected_shapes.append((label, "Rectangle"))
                cv2.putText(frame, f"{label} Rectangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
            elif len(approx) > 7:
                detected_shapes.append((label, "Circle"))
                cv2.putText(frame, f"{label} Circle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
    return detected_shapes

cap = cv2.VideoCapture(0)
object_counter_triangle_orange = 0
object_counter_rectangle_orange = 0
object_counter_circle_orange = 0
object_counter_triangle_blue = 0
object_counter_rectangle_blue = 0
object_counter_circle_blue = 0
object_counter_triangle_white = 0
object_counter_rectangle_white = 0
object_counter_circle_white = 0
line_y = 480  # y-coordinate of the line
detectFlag = 0
shape_code = 9
# Cấu hình MQTT broker
broker = "mqtt.fuvitech.vn"
port = 1883
topic = "giaphu/ctr"
# Hàm callback khi kết nối thành công
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        print("Connection successful")
    else:
        print("Connection failed")
# Tạo MQTT client
client = mqtt.Client()
# Gắn hàm callback
client.on_connect = on_connect
# Kết nối tới broker
client.connect(broker, port, 60)
# Bắt đầu vòng lặp để duy trì kết nối và xử lý các sự kiện
client.loop_start()

# Hàm đếm sản phẩm từ cv2
def count_products():
    # Giả lập dữ liệu đếm sản phẩm
    products = {"box 1": object_counter_circle_blue, "box 2": object_counter_triangle_blue, "box 3": object_counter_rectangle_blue,
                "box 4": object_counter_circle_white, "box 5": object_counter_triangle_white, "box 6": object_counter_rectangle_white,
                "box 7": object_counter_circle_orange, "box 8": object_counter_triangle_orange, "box 9": object_counter_rectangle_orange}
    return products

def send_data():
	global shape_code
	while True:
		print(shape_code)
		ser.write(f"{shape_code}\n".encode())
		#ser.read()
		time.sleep(1)

p1 = threading.Thread(target=send_data)
p1.start()

while True:
	product_counts = count_products()
	message=json.dumps(product_counts)
	client.publish(topic, message)
	#print(f"Sent message: {message}")
	_, frame = cap.read()
	height, width = frame.shape[:2]
	x, y, w, h = 900, 420, 450, 700
	frame = frame[y:y+h, x:x+w]
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
	if all_detected_shapes:
		
		for label, shape in all_detected_shapes:
			if label == "Pink":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_triangle_orange += 1
					shape_code = 0
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_rectangle_orange += 1
					shape_code = 1
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					object_counter_circle_orange += 1
					shape_code = 2
			elif label == "Yellow":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_triangle_blue += 1
					shape_code = 3
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_rectangle_blue += 1
					shape_code = 4
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					object_counter_circle_blue += 1
					shape_code = 5
			elif label == "White":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_triangle_white += 1
					shape_code = 6
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					object_counter_rectangle_white += 1
					shape_code = 7
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					object_counter_circle_white += 1
					shape_code = 8
	if not all_detected_shapes:
		detectFlag=0
		shape_code = 9
	
	height, width = frame.shape[:2]
	new_height = 320
	new_width = int((new_height / height) * width)
	resized_frame = cv2.resize(frame, (new_width, new_height))
	resized_mask = cv2.resize(combined_mask, (new_width, new_height))

	cv2.imshow("Frame", frame)
	cv2.imshow("Detection Frame", resized_mask)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break


cap.release()
cv2.destroyAllWindows()
#client.loop_stop()
#client.disconnect()
