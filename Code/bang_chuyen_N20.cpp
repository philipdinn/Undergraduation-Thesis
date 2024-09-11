#define MOTOR_PIN 14
#define IR_SENSOR_PIN 25

const int pwmFreq = 5000; //Đặt tần số PWM (điều chế độ rộng xung) là 5000 Hz. // càng cao càng mịn
const int pwmResolution = 8; //Đặt độ phân giải PWM là 8 bit (giá trị từ 0 đến 255).
const int pwmChannel = 0;  //Đặt kênh PWM là 0.

#include <WiFi.h>  // WiFi.h: Thư viện để kết nối WiFi.
#include <PubSubClient.h>  // PubSubClient.h: Thư viện để sử dụng giao thức MQTT.
#include <Arduino.h>  // Arduino.h: Thư viện cơ bản của Arduino.
#include <driver/ledc.h>  // driver/ledc.h: Thư viện điều khiển LEDC của ESP32 (PWM).


const char* ssid = "Fuvitech";  // Đặt tên mạng WiFi và mật khẩu để kết nối.
const char* password = "fuvitech.vn";


const char* mqtt_server = "mqtt.fuvitech.vn";  // Đặt địa chỉ server MQTT
const char* mqtt_topic1 = "giaphu/bangchuyen";  //bc
const char* mqtt_topic2 = "giaphu/tocdo";   //tđ


WiFiClient espClient;  // Tạo đối tượng WiFiClient để quản lý kết nối WiFi.
PubSubClient client(espClient);  // Tạo đối tượng PubSubClient để quản lý kết nối MQTT thông qua espClient.

unsigned long lastDetectionTime = 0;  // lastDetectionTime: Biến lưu thời gian lần cuối cảm biến phát hiện chuyển động.
const unsigned long timeoutDuration = 60000; // timeoutDuration: Thời gian chờ (60 giây) để xác định động cơ dừng khi không có chuyển động.


bool motorState = false;    // motorState: Trạng thái của động cơ (đang chạy hay không).
int speed = 6;              // speed: Tốc độ của động cơ, giá trị ban đầu là 6.

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());  // Chờ cho đến khi kết nối thành công, in ra địa chỉ IP khi đã kết nối.
}

void callback(char* topic, byte* message, unsigned int length) {       // Hàm callback được gọi khi có tin nhắn MQTT mới:
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");                                         // In ra thông tin về chủ đề và tin nhắn nhận được.
  String Value;                                                        // Chuyển đổi tin nhắn từ dạng byte sang chuỗi và lưu vào Value.

  for (int i = 0; i < length; i++) {     // Duyệt qua tất cả các ký tự trong tin nhắn nhận được.
    Serial.print((char)message[i]);      // In từng ký tự của tin nhắn nhận được ra màn hình Serial để kiểm tra và theo dõi.
    Value += (char)message[i];           // Tạo ra một chuỗi hoàn chỉnh từ các ký tự trong mảng message
  }
  Serial.println();
  
  if (String(topic) == mqtt_topic1) {
    Serial.print("Received value: ");
    Serial.println(Value);
    if (Value == "1") {
      motorState = true; 
    }
    if (Value == "0") {
      motorState = false; 
    }
  } else if (String(topic) == mqtt_topic2) {
    Serial.print("Received speed value: ");
    Serial.println(Value);
    speed = Value.toInt();
    Serial.print("Motor speed set to: ");
    Serial.println(speed);
  }
}

void reconnect() {        // Hàm reconnect() để kết nối lại MQTT nếu bị mất kết nối
  while (!client.connected()) {                        // Mục đích: Nếu client không kết nối, thì vòng lặp while sẽ tiếp tục chạy cho đến khi kết nối thành công.
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {               //  Thử kết nối tới máy chủ MQTT bằng cách gọi client.connect("ESP32Client")     // "ESP32Client": Đây là ID của client
      Serial.println("connected");
      client.subscribe(mqtt_topic1);                   // Đăng ký (subscribe) vào hai chủ đề (mqtt_topic1 và mqtt_topic2) để nhận tin nhắn từ máy chủ MQTT.
      client.subscribe(mqtt_topic2); 
    } else {
      Serial.print("failed, rc=");                     // Ý nghĩa: Nếu kết nối không thành công (client.connect() trả về false):
      Serial.print(client.state());                    // In ra thông báo "failed, rc=" trên cổng Serial, theo sau là trạng thái lỗi (client.state()).
      Serial.println(" try again in 5 seconds");       // Gọi delay(5000) để chờ 5 giây trước khi thử kết nối lại.
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  setup_wifi();                         // Gọi setup_wifi() để kết nối WiFi.        
  client.setServer(mqtt_server, 1883);  // Thiết lập server và hàm callback cho MQTT.
  client.setCallback(callback);         // Định nghĩa hàm callback để xử lý tin nhắn MQTT khi nhận được.
  
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);   //  Thiết lập kênh PWM với tần số và độ phân giải đã định nghĩa trước.
  ledcAttachPin(MOTOR_PIN, pwmChannel);            // Cho phép điều khiển động cơ được kết nối với chân MOTOR_PIN bằng cách sử dụng kênh PWM đã cấu hình.
  
  pinMode(IR_SENSOR_PIN, INPUT);
}

void loop() {
  if (!client.connected()) {  // Kiểm tra và kết nối lại MQTT nếu bị mất kết nối.
    reconnect();              // Nếu client không kết nối, gọi hàm reconnect() để kết nối lại.
  }
  client.loop();              // Duy trì kết nối MQTT và xử lý bất kỳ tin nhắn nào nhận được.

  int sensorValue = digitalRead(IR_SENSOR_PIN);
  Serial.print("IR Sensor Value: ");
  Serial.println(sensorValue);

  if (sensorValue == LOW) { 
    lastDetectionTime = millis(); 
    motorState = true; 
  } else {
    
    if (millis() - lastDetectionTime > timeoutDuration) {
      motorState = false; 
    }
  }

  
  if (motorState) {
    ledcWrite(pwmChannel, speed);
  } else {                            // Nếu động cơ tắt (motorState == false)
    ledcWrite(pwmChannel, 0);         // Đặt giá trị PWM cho động cơ bằng 0 (tắt động cơ).
  }

 
  delay(50);
}
