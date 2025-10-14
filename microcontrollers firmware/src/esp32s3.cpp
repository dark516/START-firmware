#include <WiFi.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const char* ssid = "BV9300 Pro";
const char* password = "1aaaaaaa";

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

WiFiServer server(3333);
WiFiClient client;

SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7); // CS = GPIO7
struct can_frame canMsg;

// Buffer for WiFi incoming commands
String wifiBuffer = "";

int convertedX(int angle) {
  if (angle <= 180) return angle;
  return angle - 360;
}

// Function to send command to Arduino via CAN
void sendCommandToArduino(float linear_x, float angular_z) {
  struct can_frame cmdFrame;
  cmdFrame.can_id = 0x200;  // Command ID for Arduino
  cmdFrame.can_dlc = 8;
  
  // Pack linear and angular velocities as floats
  memcpy(&cmdFrame.data[0], &linear_x, 4);
  memcpy(&cmdFrame.data[4], &angular_z, 4);
  
  if (mcp2515.sendMessage(&cmdFrame) == MCP2515::ERROR_OK) {
    Serial.printf("Command sent to Arduino: lin=%.3f, ang=%.3f\n", linear_x, angular_z);
  } else {
    Serial.println("Failed to send command to Arduino");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32S3 WiFi Bridge Starting ===");

  // WiFi connection
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n✅ WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  
  // Start WiFi server
  server.begin();
  Serial.printf("✅ WiFi server started on port 3333\n");

  // Initialize MCP2515 (CAN controller)
  Serial.println("Initializing CAN controller...");
  mySpi.begin(4, 6, 5, 7); // SCK=4, MISO=6, MOSI=5, CS=7
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("✅ CAN controller initialized");

  // Initialize BNO055 IMU
  Serial.println("Initializing IMU sensor...");
  Wire.begin(16, 15);
  if (!bno.begin()) {
    Serial.println("❌ Failed to initialize BNO055!");
  } else {
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("✅ IMU sensor initialized");
  }
  
  delay(1000);
  Serial.println("🚀 ESP32S3 WiFi Bridge ready!");
  Serial.println(WiFi.localIP());
  Serial.println("Waiting for ROS2 connection...");
}

void loop() {
  // Accept client connection
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
      wifiBuffer = ""; // Clear buffer for new client
    }
  }

  // Handle incoming WiFi commands from ROS2
  if (client && client.connected() && client.available()) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        // Process complete command
        wifiBuffer.trim();
        if (wifiBuffer.length() > 0) {
          int spaceIndex = wifiBuffer.indexOf(' ');
          if (spaceIndex > 0) {
            float linear_x = wifiBuffer.substring(0, spaceIndex).toFloat();
            float angular_z = wifiBuffer.substring(spaceIndex + 1).toFloat();
            
            Serial.printf("Received command: linear_x=%.3f, angular_z=%.3f\n", linear_x, angular_z);
            sendCommandToArduino(linear_x, angular_z);
          }
        }
        wifiBuffer = ""; // Clear buffer
      } else if (c != '\r') {
        wifiBuffer += c;
      }
    }
  }

  // Receive data from Arduino via CAN and send to ROS2 via WiFi
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Serial.println("Received CAN data from Arduino");
    if (canMsg.can_id == 0x100 && canMsg.can_dlc == 8) {
      int32_t left_ticks, right_ticks;
      memcpy(&left_ticks, &canMsg.data[0], 4);
      memcpy(&right_ticks, &canMsg.data[4], 4);

      // Get IMU data
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      
      int yaw = convertedX((int)euler.x());
      float x_accel = accel.y(); // X acceleration (forward/backward)
      
      Serial.printf("Sensor data - L: %ld, R: %ld, Yaw: %d, X_Accel: %.3f\n", 
                    left_ticks, right_ticks, yaw, x_accel);
      
      // Send data to ROS2 via WiFi
      if (client && client.connected()) {
        char buffer[80];
        int len = snprintf(buffer, sizeof(buffer), "%ld %ld %d %.3f\n",
                           left_ticks, right_ticks, yaw, x_accel);
        client.write((uint8_t*)buffer, len);
      }
    }
  }
}
