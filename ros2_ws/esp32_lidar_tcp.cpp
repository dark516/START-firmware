#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <HardwareSerial.h>

// Version
const char* FIRMWARE_VERSION = "v1.1-TCP";

// WiFi credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// TCP settings
WiFiServer server(3333);
WiFiClient client;
bool clientConnected = false;

// LIDAR settings
#define LIDAR_BAUDRATE 460800
#define LIDAR_RX_PIN 11  // Green (RX lidar) - ESP32 receives from LIDAR TX
#define LIDAR_TX_PIN 12  // Yellow (TX lidar) - ESP32 sends to LIDAR RX
HardwareSerial LidarSerial(1);

// Raw data streaming buffer
uint8_t streamBuffer[512];
int bufferIndex = 0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 10; // Send every 10ms or when buffer fills

// Statistics
unsigned long totalBytesSent = 0;
unsigned long totalPacketsSent = 0;
unsigned long lastStatsTime = 0;
const unsigned long STATS_INTERVAL_MS = 5000; // Print stats every 5 seconds

// Raw data display
bool showRawData = true;
unsigned long lastRawDataTime = 0;
const unsigned long RAW_DATA_INTERVAL_MS = 1000; // Show raw data every second

// LIDAR commands
uint8_t STOP_CMD[] = {0xA5, 0x25};
uint8_t RESET_CMD[] = {0xA5, 0x40};
uint8_t HEALTH_CMD[] = {0xA5, 0x52};
uint8_t START_SCAN_CMD[] = {0xA5, 0x20};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ESP32 LIDAR TCP Raw Data Streaming ===");
    Serial.printf("Version: %s\n", FIRMWARE_VERSION);
    Serial.println("Initializing...");
    
    // Initialize LIDAR serial
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
    
    // Start TCP server
    server.begin();
    Serial.printf("TCP server started on port 3333\n");
    Serial.println("Waiting for client connection...");
    
    // Initialize LIDAR
    initializeLidar();
    
    Serial.println("Setup complete - ready for raw data streaming!");
    Serial.println("Raw data format: [HEX] followed by ASCII if printable");
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
    
    // Read and display health response
    Serial.println("LIDAR Health Response:");
    unsigned long healthStart = millis();
    while (millis() - healthStart < 500) {
        if (LidarSerial.available()) {
            uint8_t byte = LidarSerial.read();
            Serial.printf("0x%02X ", byte);
        }
    }
    Serial.println();
    
    // Start scanning
    Serial.println("Starting LIDAR scan...");
    LidarSerial.write(START_SCAN_CMD, 2);
    delay(100);
    
    Serial.println("LIDAR initialization complete");
}

void handleTCPConnection() {
    if (!clientConnected) {
        client = server.available();
        if (client) {
            clientConnected = true;
            Serial.printf("Client connected from: %s\n", client.remoteIP().toString().c_str());
        }
    } else {
        // Check if client is still connected
        if (!client.connected()) {
            clientConnected = false;
            client.stop();
            Serial.println("Client disconnected");
        }
    }
}

void sendRawDataBuffer() {
    if (bufferIndex > 0 && clientConnected && client.connected()) {
        size_t bytesSent = client.write(streamBuffer, bufferIndex);
        
        if (bytesSent == bufferIndex) {
            totalBytesSent += bufferIndex;
            totalPacketsSent++;
        } else {
            Serial.printf("TCP send incomplete: sent %d of %d bytes\n", bytesSent, bufferIndex);
        }
        
        bufferIndex = 0;
    }
}

void printRawDataSample() {
    unsigned long now = millis();
    if (now - lastRawDataTime >= RAW_DATA_INTERVAL_MS && bufferIndex > 0) {
        Serial.println("=== Raw LIDAR Data Sample ===");
        Serial.print("HEX: ");
        for (int i = 0; i < min(bufferIndex, 32); i++) {  // Show first 32 bytes
            Serial.printf("%02X ", streamBuffer[i]);
        }
        Serial.println();
        
        Serial.print("ASCII: ");
        for (int i = 0; i < min(bufferIndex, 32); i++) {
            char c = streamBuffer[i];
            if (c >= 32 && c <= 126) {  // Printable ASCII
                Serial.print(c);
            } else {
                Serial.print('.');
            }
        }
        Serial.println();
        Serial.printf("Buffer size: %d bytes\n", bufferIndex);
        Serial.println("============================");
        
        lastRawDataTime = now;
    }
}

void printStatistics() {
    unsigned long now = millis();
    if (now - lastStatsTime >= STATS_INTERVAL_MS) {
        float elapsedSec = (now - lastStatsTime) / 1000.0;
        float bytesPerSec = totalBytesSent / elapsedSec;
        float packetsPerSec = totalPacketsSent / elapsedSec;
        
        Serial.printf("=== TCP Streaming Stats ===\n");
        Serial.printf("Connected: %s\n", clientConnected ? "YES" : "NO");
        Serial.printf("Bytes sent: %lu (%.1f B/s)\n", totalBytesSent, bytesPerSec);
        Serial.printf("Packets sent: %lu (%.1f pkt/s)\n", totalPacketsSent, packetsPerSec);
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.println("===========================");
        
        totalBytesSent = 0;
        totalPacketsSent = 0;
        lastStatsTime = now;
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "STOP") {
            LidarSerial.write(STOP_CMD, 2);
            Serial.println("LIDAR STOP command sent");
        } else if (command == "START") {
            LidarSerial.write(START_SCAN_CMD, 2);
            Serial.println("LIDAR START command sent");
        } else if (command == "RESET") {
            LidarSerial.write(RESET_CMD, 2);
            Serial.println("LIDAR RESET command sent");
        } else if (command == "HEALTH") {
            LidarSerial.write(HEALTH_CMD, 2);
            Serial.println("LIDAR HEALTH command sent");
        } else if (command == "RAW") {
            showRawData = !showRawData;
            Serial.printf("Raw data display: %s\n", showRawData ? "ON" : "OFF");
        } else if (command == "HELP") {
            Serial.println("Available commands:");
            Serial.println("  START  - Start LIDAR scanning");
            Serial.println("  STOP   - Stop LIDAR scanning");
            Serial.println("  RESET  - Reset LIDAR");
            Serial.println("  HEALTH - Get LIDAR health");
            Serial.println("  RAW    - Toggle raw data display");
            Serial.println("  HELP   - Show this help");
        }
    }
}

void loop() {
    unsigned long now = millis();
    
    // Handle TCP connections
    handleTCPConnection();
    
    // Handle serial commands
    handleSerialCommands();
    
    // Read LIDAR data and stream raw bytes immediately
    while (LidarSerial.available()) {
        uint8_t byte = LidarSerial.read();
        streamBuffer[bufferIndex++] = byte;
        
        // Send buffer if full or time interval reached
        if (bufferIndex >= sizeof(streamBuffer) || 
            (now - lastSendTime >= SEND_INTERVAL_MS && bufferIndex > 0)) {
            sendRawDataBuffer();
            lastSendTime = now;
        }
    }
    
    // Send remaining data if time interval reached
    if (now - lastSendTime >= SEND_INTERVAL_MS && bufferIndex > 0) {
        sendRawDataBuffer();
        lastSendTime = now;
    }
    
    // Show raw data sample for debugging
    if (showRawData) {
        printRawDataSample();
    }
    
    // Print statistics periodically
    printStatistics();
    
    // Small delay to prevent overwhelming the system
    delay(1);
}