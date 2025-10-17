#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

// Version
const char* FIRMWARE_VERSION = "v1.0";

// WiFi credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// UDP settings
WiFiUDP udp;
const int UDP_PORT = 3333;
IPAddress clientIP;
const int CLIENT_PORT = 3334;
bool clientRegistered = false;

// LIDAR settings
#define LIDAR_BAUDRATE 460800
#define LIDAR_RX_PIN 11  // Green (RX lidar) - ESP32 receives from LIDAR TX
#define LIDAR_TX_PIN 12  // Yellow (TX lidar) - ESP32 sends to LIDAR RX
HardwareSerial LidarSerial(1);

// Streaming buffer - very small for immediate transmission
uint8_t streamBuffer[10];
int bufferIndex = 0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 2; // Send every 2ms or when buffer fills

// Statistics
unsigned long totalBytesSent = 0;
unsigned long totalPacketsSent = 0;
unsigned long lastStatsTime = 0;
const unsigned long STATS_INTERVAL_MS = 5000; // Print stats every 5 seconds

// LIDAR commands
uint8_t STOP_CMD[] = {0xA5, 0x25};
uint8_t RESET_CMD[] = {0xA5, 0x40};
uint8_t HEALTH_CMD[] = {0xA5, 0x52};
uint8_t START_SCAN_CMD[] = {0xA5, 0x20};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ESP32 LIDAR Streaming Firmware ===");
    Serial.printf("Version: %s\n", FIRMWARE_VERSION);
    Serial.println("Initializing...");
    
    // Initialize LIDAR serial with correct pin order (matching working code)
    LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.printf("LIDAR Serial initialized: RX=%d, TX=%d, Baud=%d\n", 
                  LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
    
    // Initialize WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    
    // Start UDP
    udp.begin(UDP_PORT);
    Serial.printf("UDP server started on port %d\n", UDP_PORT);
    
    // Initialize LIDAR
    initializeLidar();
    
    Serial.println("Setup complete - ready for streaming!");
}

void initializeLidar() {
    Serial.println("Initializing LIDAR...");
    
    // Stop any ongoing scan
    LidarSerial.write(STOP_CMD, 2);
    delay(100);
    
    // Reset LIDAR
    LidarSerial.write(RESET_CMD, 2);
    delay(1000);
    
    // Check health
    LidarSerial.write(HEALTH_CMD, 2);
    delay(100);
    
    // Read and discard health response
    unsigned long healthStart = millis();
    while (millis() - healthStart < 500) {
        if (LidarSerial.available()) {
            LidarSerial.read();
        }
    }
    
    // Start scanning
    LidarSerial.write(START_SCAN_CMD, 2);
    delay(100);
    
    Serial.println("LIDAR initialization complete");
}

void handleUDPRegistration() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char buffer[64];
        int len = udp.read(buffer, sizeof(buffer) - 1);
        buffer[len] = 0;
        
        if (strcmp(buffer, "REGISTER") == 0) {
            clientIP = udp.remoteIP();
            clientRegistered = true;
            Serial.printf("Client registered: %s\n", clientIP.toString().c_str());
            
            // Send acknowledgment
            udp.beginPacket(clientIP, CLIENT_PORT);
            udp.write((uint8_t*)"ACK", 3);
            udp.endPacket();
        }
    }
}

void sendStreamBuffer() {
    if (bufferIndex > 0 && clientRegistered) {
        udp.beginPacket(clientIP, CLIENT_PORT);
        udp.write(streamBuffer, bufferIndex);
        int result = udp.endPacket();
        
        if (result == 1) {
            totalBytesSent += bufferIndex;
            totalPacketsSent++;
        } else {
            Serial.printf("UDP send error: %d\n", result);
        }
        
        bufferIndex = 0;
    }
}

void printStatistics() {
    unsigned long now = millis();
    if (now - lastStatsTime >= STATS_INTERVAL_MS) {
        float bytesPerSec = totalBytesSent / ((now - lastStatsTime) / 1000.0);
        Serial.printf("Stats: %lu bytes sent, %lu packets, %.1f B/s\n", 
                      totalBytesSent, totalPacketsSent, bytesPerSec);
        
        totalBytesSent = 0;
        totalPacketsSent = 0;
        lastStatsTime = now;
    }
}

void loop() {
    unsigned long now = millis();
    
    // Handle UDP registration
    handleUDPRegistration();
    
    // Read LIDAR data byte by byte and stream immediately
    while (LidarSerial.available()) {
        uint8_t byte = LidarSerial.read();
        streamBuffer[bufferIndex++] = byte;
        
        // Send buffer if full or time interval reached
        if (bufferIndex >= sizeof(streamBuffer) || 
            (now - lastSendTime >= SEND_INTERVAL_MS && bufferIndex > 0)) {
            sendStreamBuffer();
            lastSendTime = now;
        }
    }
    
    // Send remaining data if time interval reached
    if (now - lastSendTime >= SEND_INTERVAL_MS && bufferIndex > 0) {
        sendStreamBuffer();
        lastSendTime = now;
    }
    
    // Print statistics periodically
    printStatistics();
    
    // Small delay to prevent overwhelming the system
    delay(1);
}