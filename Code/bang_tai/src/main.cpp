#define SENSOR_PIN 25
#define MOTOR_PIN 14
#define IR_SENSOR_PIN 25

const int pwmFreq = 5000; 
const int pwmResolution = 8; 
const int pwmChannel = 0;

#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <driver/ledc.h>


const char* ssid = "Fuvitech";
const char* password = "fuvitech.vn";


const char* mqtt_server = "mqtt.fuvitech.vn";
const char* mqtt_topic1 = "giaphu/bangchuyen";
const char* mqtt_topic2 = "giaphu/tocdo"; 


WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastDetectionTime = 0;
const unsigned long timeoutDuration = 60000; // 60 seconds


bool motorState = false; 
int speed = 6; 

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
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String Value;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    Value += (char)message[i];
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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic1);
      client.subscribe(mqtt_topic2); 
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN, pwmChannel);
  
  pinMode(IR_SENSOR_PIN, INPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int sensorValue = digitalRead(IR_SENSOR_PIN);
  Serial.print("IR Sensor Value: ");
  Serial.println(sensorValue);

  if (sensorValue == LOW) { 
    lastDetectionTime = millis(); 
    motorState = true; 
  } else {
    
    if (millis() - lastDetectionTime > timeoutDuration) {
      motorState = false; 
      speed = 128; 
    }
  }

  
  if (motorState) {
    ledcWrite(pwmChannel, speed);
  } else {
    ledcWrite(pwmChannel, 0);
  }

 
  delay(50);
}
