import cv2  # xử lý, nhận diện h/ả
import numpy as np  # mảng, ma trận, tính toán, xử lý h/ả
import time  # độ trễ, tg hiện tại, tg thực thi 
import json  # lưu trữ và truyền tải dạng json
import paho.mqtt.client as mqtt  # kết nối nối Broker của MQTT
import serial
import threading  # chạy các tác vụ đồng thời (xử lý hả và gửi dữ liệu qua mạng)
try:
    ser = serial.Serial('/dev/ttyUSB4', 115200, timeout=1)  # Kết nối với cổng serial, tốc độ 115200 baud, timeout 1 giây
except:
    ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Thử kết nối với cổng khác nếu cổng trên không thành công
time.sleep(2)
        
def detect_and_label_shapes(contours, frame, label, color):
    detected_shapes = []                   # Khởi tạo danh sách detected_shapes rỗng để lưu các hình dạng được phát hiện cùng với nhãn của chúng.
    for contour in contours:               # Vòng lặp for để duyệt qua từng đường viền (contour) trong danh sách contours.
        area = cv2.contourArea(contour)    # Tính diện tích của đường viền
        if area > 10000:        # Kiểm tra xem diện tích của đường viền có lớn hơn 10000 hay không. Chỉ xử lý các đường viền có diện tích lớn hơn giá trị này để bỏ qua các đối tượng nhỏ không quan trọng.
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)      # -1 vẽ đường viền trong frame, xanh lá, độ dày 2 pixel
            perimeter = cv2.arcLength(contour, True)                    # Tính chu vi của đường viền
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)  # Tìm đa giác xấp xỉ của đường viền với độ chính xác là 3% của chu vi. Hàm cv2.approxPolyDP dùng để đơn giản hóa hình dạng của đường viền.
            if len(approx) == 3:
                detected_shapes.append((label, "Triangle"))
                cv2.putText(frame, f"{label} Triangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4) # 1 là cỡ chũ, 4 là độ dày
            elif len(approx) == 4:
                detected_shapes.append((label, "Rectangle"))
                cv2.putText(frame, f"{label} Rectangle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
            elif len(approx) > 7:
                detected_shapes.append((label, "Circle"))
                cv2.putText(frame, f"{label} Circle", (x + 120, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 4)
    return detected_shapes    # Trả về danh sách các hình đã nhận diện

cap = cv2.VideoCapture(0)    # Mở camera mặc định
object_counter_triangle_pink = 0
object_counter_rectangle_pink = 0
object_counter_circle_pink = 0
object_counter_triangle_yellow = 0
object_counter_rectangle_yellow = 0
object_counter_circle_yellow = 0
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
giaphu_control_control = True
def on_message(client, userdata, msg):
    global giaphu_control_control
    global object_counter_triangle_pink
    global object_counter_rectangle_pink
    global object_counter_circle_pink
    global object_counter_triangle_yellow
    global object_counter_rectangle_yellow
    global object_counter_circle_yellow
    global object_counter_triangle_white
    global object_counter_rectangle_white
    global object_counter_circle_white
    '''
    giaphu/control b'false'
    giaphu/control b'true'
    '''
    print(msg.topic + " " + str(msg.payload))
    if msg.topic == 'giaphu/control':
        if msg.payload == b'false':
            # Perform actions when the message "giaphu/control" with payload "false" is received
            print("Received 'false' message on 'giaphu/control' topic")
            giaphu_control_control = False
            # Add your desired actions here
        elif msg.payload == b'true':
            # Perform actions when the message "giaphu/control" with payload "true" is received
            print("Received 'true' message on 'giaphu/control' topic")
            # Add your desired actions here
            giaphu_control_control = True
    elif msg.topic == 'giaphu/count':
        print('giaphu/count')
        object_counter_triangle_pink = 0
        object_counter_rectangle_pink = 0
        object_counter_circle_pink = 0
        object_counter_triangle_yellow = 0
        object_counter_rectangle_yellow = 0
        object_counter_circle_yellow = 0
        object_counter_triangle_white = 0
        object_counter_rectangle_white = 0
        object_counter_circle_white = 0
        product_counts = count_products()
        print(product_counts)
        message=json.dumps(product_counts)
        client.publish(topic, message)
    else:
        # Handle other messages if necessary
        print("Received message on topic: " + msg.topic + ", payload: " + str(msg.payload))
# Hàm callback khi kết nối thành công
def on_connect(client, userdata, flags, rc):    # Hàm xử lý khi kết nối với MQTT Broker thành công
    print(f"Connected with result code {rc}")   # In ra mã kết nối thành công
    if rc == 0:
        print("Connection successful")
        client.publish("giaphu/start",True)
        client.subscribe("giaphu/count")
        client.subscribe("giaphu/control")
    else:
        print("Connection failed")
# Tạo MQTT client
client = mqtt.Client()
# Gắn hàm callback
client.on_connect = on_connect
client.on_message = on_message
# Kết nối tới broker
client.connect(broker, port, 60)
# Bắt đầu vòng lặp để duy trì kết nối và xử lý các sự kiện
client.loop_start()

# Hàm đếm sản phẩm từ cv2
def count_products():
    # Giả lập dữ liệu đếm sản phẩm
    products = {"circle_yellow": object_counter_circle_yellow, "triangle_yellow": object_counter_triangle_yellow, "rectangle_yellow": object_counter_rectangle_yellow,
                "circle_white": object_counter_circle_white, "triangle_white": object_counter_triangle_white, "rectangle_white": object_counter_rectangle_white,
                "circle_pink": object_counter_circle_pink, "triangle_pink": object_counter_triangle_pink, "rectangle_pink": object_counter_rectangle_pink}
    return products

shape_code_list =[]
def check_last_three(shape_code_list):    # kiểm tra xem ba giá trị cuối cùng trong danh sách 
    if len(shape_code_list) < 4:          # Điều này đảm bảo rằng phải có ít nhất 4 phần tử để kiểm tra.
        return False
    
    last_three = shape_code_list[-3:]     #  là danh sách chứa ba phần tử cuối cùng
    if len(set(last_three)) == 1 and last_three[0] != shape_code_list[-4]:  # Kiểm tra xem ba phần tử cuối có giống nhau hay không bằng cách chuyển chúng thành tập hợp (set). Nếu tất cả các phần tử giống nhau, tập hợp sẽ có độ dài là 1.
        return True                       # Nếu cả hai điều kiện trên đều đúng, hàm trả về True; ngược lại, hàm trả về False.
    else:
        return False
	
def send_data():
	global shape_code_list
	global shape_code
	global object_counter_triangle_pink
	global object_counter_rectangle_pink
	global object_counter_circle_pink
	global object_counter_triangle_yellow
	global object_counter_rectangle_yellow
	global object_counter_circle_yellow
	global object_counter_triangle_white
	global object_counter_rectangle_white
	global object_counter_circle_white
	
	while True:
		print(shape_code)
		shape_code_list.append(shape_code)
		if check_last_three(shape_code_list):
			if shape_code == 0:
				object_counter_triangle_pink += 1
			if shape_code == 1:
				object_counter_rectangle_pink += 1
			if shape_code == 2:
				object_counter_circle_pink += 1

			if shape_code == 3:
				object_counter_triangle_yellow += 1
			if shape_code == 4:
				object_counter_rectangle_yellow += 1
			if shape_code == 5:
				object_counter_circle_yellow += 1

			if shape_code == 6:
				object_counter_triangle_white += 1
			if shape_code == 7:
				object_counter_rectangle_white += 1
			if shape_code == 8:
				object_counter_circle_white += 1
			product_counts = count_products()
			print(product_counts)
			message=json.dumps(product_counts)
			client.publish(topic, message)
		ser.write(f"{shape_code}\n".encode())
		#ser.read()
		time.sleep(1)

p1 = threading.Thread(target=send_data)
p1.start()

while True:  # Vòng lặp vô hạn để liên tục đọc khung hình từ camera
	if not giaphu_control_control:
		print('Stop')
		detectFlag=0
		shape_code = 10
		time.sleep(2)
		continue
	#print(f"Sent message: {message}")
	_, frame = cap.read()                       # đọc frame từ camera
	height, width = frame.shape[:2]
	x, y, w, h = 900, 420, 450, 700
	frame = frame[y:y+h, x:x+w]
	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	low_pink = np.array([155, 100, 180])
	high_pink = np.array([178, 200, 255])
	low_yellow = np.array([20, 128, 180])
	high_yellow = np.array([50, 240, 255])
	low_white = np.array([0, 0, 150])
	high_white = np.array([180, 100, 255])

	pink_mask = cv2.inRange(hsv_frame, low_pink, high_pink)
	yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)
	white_mask = cv2.inRange(hsv_frame, low_white, high_white)

	combined_mask = cv2.bitwise_or(pink_mask, white_mask)
	combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)

	contours_pink, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_white, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	detected_pink_shapes = detect_and_label_shapes(contours_pink, frame, "Pink", (119, 55, 230))
	detected_yellow_shapes = detect_and_label_shapes(contours_yellow, frame, "Yellow", (55, 249, 249))
	detected_white_shapes = detect_and_label_shapes(contours_white, frame, "White", (255, 255, 255))
	all_detected_shapes = detected_pink_shapes + detected_yellow_shapes + detected_white_shapes

	if all_detected_shapes:
		for label, shape in all_detected_shapes:
			if label == "Pink":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 0
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 1
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 2
			elif label == "Yellow":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 3
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 4
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 5
			elif label == "White":
				if shape == "Triangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 6
				elif shape == "Rectangle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 7
				elif shape == "Circle" and detectFlag == 0:
					detectFlag = 1
					shape_code = 8
	
	if not all_detected_shapes:  # Nếu không có hình dạng nào được nhận diện, đặt detectFlag và shape_code về giá trị mặc định.
		detectFlag=0
		shape_code = 9
	
	height, width = frame.shape[:2]     # Lấy chiều cao và chiều rộng của khung hình (Ví dụ: frame.shape có thể trả về (480, 640, 3), có nghĩa là khung hình có chiều cao 480 pixel, chiều rộng 640 pixel, và có 3 kênh màu (BGR).)
	new_height = 320   # Pixel         # Đặt chiều cao mới cho khung hình
	new_width = int((new_height / height) * width)           # Tính toán chiều rộng mới dựa trên tỷ lệ chiều cao và chiều rộng ban đầu
	resized_frame = cv2.resize(frame, (new_width, new_height))
	resized_mask = cv2.resize(combined_mask, (new_width, new_height))
	frame = cv2.resize(frame, (new_width, new_height))
	
	cv2.imshow("Frame", frame)                   # Hiển thị khung hình "Frame" đã thay đổi kích thước
	cv2.imshow("Detection Frame", resized_mask)

	if cv2.waitKey(1) & 0xFF == ord('q'):     # Nếu nhấn phím 'q' thì thoát khỏi vòng lặp
		break


cap.release()            # Giải phóng camera
cv2.destroyAllWindows()  # Đóng tất cả các cửa sổ hiển thị
#client.loop_stop()      # Dừng vòng lặp xử lý của MQTT Client
#client.disconnect()     # Ngắt kết nối với MQTT Broker
