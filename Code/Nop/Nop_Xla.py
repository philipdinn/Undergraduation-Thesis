import cv2 
import numpy as np  
import json  
import paho.mqtt.client as mqtt  
import serial
import threading 
try:
    ser = serial.Serial('/dev/ttyUSB4', 115200, timeout=1)  
except:
    ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  
        
def detect_and_label_shapes(contours, frame, label, color):
    detected_shapes = []                  
    for contour in contours:               
        area = cv2.contourArea(contour)   
        if area > 10000:       
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)     
            perimeter = cv2.arcLength(contour, True)                  
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True) 
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
object_counter_triangle_pink = 0
object_counter_rectangle_pink = 0
object_counter_circle_pink = 0
object_counter_triangle_yellow = 0
object_counter_rectangle_yellow = 0
object_counter_circle_yellow = 0
object_counter_triangle_white = 0
object_counter_rectangle_white = 0
object_counter_circle_white = 0

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
            print("Received 'false' message on 'giaphu/control' topic")
            giaphu_control_control = False
        elif msg.payload == b'true':
            print("Received 'true' message on 'giaphu/control' topic")
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
        print("Received message on topic: " + msg.topic + ", payload: " + str(msg.payload))
def on_connect(client, userdata, flags, rc):   
    print(f"Connected with result code {rc}")   
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

def count_products():
    products = {"circle_yellow": object_counter_circle_yellow, "triangle_yellow": object_counter_triangle_yellow, "rectangle_yellow": object_counter_rectangle_yellow,
                "circle_white": object_counter_circle_white, "triangle_white": object_counter_triangle_white, "rectangle_white": object_counter_rectangle_white,
                "circle_pink": object_counter_circle_pink, "triangle_pink": object_counter_triangle_pink, "rectangle_pink": object_counter_rectangle_pink}
    return products

shape_code_list =[]
def check_last_three(shape_code_list):   
    if len(shape_code_list) < 4:         
        return False
    
    last_three = shape_code_list[-3:]    
    if len(set(last_three)) == 1 and last_three[0] != shape_code_list[-4]:  
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
		time.sleep(1)

p1 = threading.Thread(target=send_data)
p1.start()

while True: 
	if not giaphu_control_control:
		print('Stop')
		detectFlag=0
		shape_code = 10
		time.sleep(2)
		continue
	_, frame = cap.read()                      
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
	
	if not all_detected_shapes:
		detectFlag=0
		shape_code = 9
	
	height, width = frame.shape[:2]     
	new_height = 320  
	new_width = int((new_height / height) * width)         
	resized_frame = cv2.resize(frame, (new_width, new_height))
	resized_mask = cv2.resize(combined_mask, (new_width, new_height))
	frame = cv2.resize(frame, (new_width, new_height))
	
	cv2.imshow("Frame", frame)              
	cv2.imshow("Detection Frame", resized_mask)

	if cv2.waitKey(1) & 0xFF == ord('q'):   
		break


cap.release()      
cv2.destroyAllWindows()  

