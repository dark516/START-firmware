#include <WiFi.h>
#include <WiFiUdp.h>
#include <USB.h>
#include <USBCDC.h>

USBCDC USBSerial;

#pragma pack(push, 1)
struct EchoData {
  uint8_t received_byte;
  uint8_t response_byte;
  uint32_t timestamp;
  uint8_t packet_count;
};
#pragma pack(pop)

// WiFi настройки
const char* ssid = "s548-poligon";
const char* password = "Bwog4581";

// UDP настройки
WiFiUDP udp;
const uint16_t UDP_PORT = 3333;

// TCP настройки с автоопределением IP
WiFiClient user;
const int tcp_port = 8888;
IPAddress pc_ip;  // Автоопределяется из UDP
bool tcp_enabled = false;

// Переменные
EchoData echo_data;
bool sending_mode = false;
unsigned long timer = 0, last_send = 0;
unsigned long packet_counter = 0;
bool led = false;

void usb_init() {
  USB.begin();
  USBSerial.begin(115200);
}

void wifi_init() {
  WiFi.disconnect();
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  WiFi.begin(ssid, password);

  USBSerial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    USBSerial.print(".");
  }
  USBSerial.println("\nConnected!");
  USBSerial.printf("ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
}

void tcp_connection_attempt() {
  if (pc_ip == IPAddress(0, 0, 0, 0)) {
    USBSerial.println("TCP: Waiting for PC IP from UDP...");
    return;
  }
  
  USBSerial.printf("TCP: Attempting connection to %s:%d...\n", pc_ip.toString().c_str(), tcp_port);
  
  if (user.connect(pc_ip, tcp_port)) {
    USBSerial.printf("✅ TCP connected to %s:%d\n", pc_ip.toString().c_str(), tcp_port);
    user.print("ESP32_ECHO_READY\n");
    tcp_enabled = true;
  } else {
    USBSerial.printf("❌ TCP connection failed to %s:%d\n", pc_ip.toString().c_str(), tcp_port);
    tcp_enabled = false;
  }
}

void udp_init() {
  if (udp.begin(UDP_PORT)) {
    USBSerial.printf("✅ UDP server started on port %d\n", UDP_PORT);
  } else {
    USBSerial.println("❌ UDP server failed to start");
  }
}

void handle_udp_echo() {
  int packetSize = udp.parsePacket();
  
  if (packetSize > 0) {
    // Запоминаем IP отправителя для TCP подключения
    IPAddress senderIP = udp.remoteIP();
    if (pc_ip != senderIP) {
      pc_ip = senderIP;
      USBSerial.printf("🔍 PC IP detected: %s (from UDP)\n", pc_ip.toString().c_str());
      
      // Пытаемся подключиться к TCP
      if (!tcp_enabled) {
        tcp_connection_attempt();
      }
    }
    
    // Читаем входящий байт
    uint8_t incomingByte;
    int bytesRead = udp.read(&incomingByte, 1);
    
    if (bytesRead == 1) {
      packet_counter++;
      
      uint16_t senderPort = udp.remotePort();
      
      // Отправляем эхо обратно через UDP
      udp.beginPacket(senderIP, senderPort);
      udp.write(&incomingByte, 1);
      
      if (udp.endPacket()) {
        // Успешно отправлено
        if (packet_counter % 10 == 0) {
          USBSerial.printf("Echo #%lu: byte %d -> %s:%d\n", 
                          packet_counter, incomingByte, 
                          senderIP.toString().c_str(), senderPort);
        }
        
        // Также отправляем статистику через TCP (если подключен)
        if (tcp_enabled && user.connected()) {
          echo_data.received_byte = incomingByte;
          echo_data.response_byte = incomingByte;
          echo_data.timestamp = millis();
          echo_data.packet_count = packet_counter % 256;
          
          user.write((uint8_t*)&echo_data, sizeof(echo_data));
        }
        
      } else {
        USBSerial.printf("Failed to send echo for byte %d\n", incomingByte);
      }
    }
  }
}

void handle_tcp_commands() {
  if (tcp_enabled && user.connected() && user.available()) {
    String task = user.readStringUntil('\n');
    
    USBSerial.printf("TCP command received: '%s'\n", task.c_str());
    
    if (task == "die") {
      USBSerial.println("Restarting...");
      ESP.restart();
    } else if (task == "work") {
      sending_mode = true;
      USBSerial.println("Work mode - UDP echo active");
    } else if (task == "rest") {
      sending_mode = false;
      USBSerial.println("Rest mode - UDP echo paused");
    } else if (task == "stats") {
      USBSerial.printf("Packets processed: %lu\n", packet_counter);
    }
  }
}

void send_periodic_data() {
  if (sending_mode && last_send + 2000 < millis()) {
    last_send = millis();
    
    if (tcp_enabled && user.connected()) {
      // Отправляем периодическую статистику
      echo_data.received_byte = 0xFF;  // Маркер статистики
      echo_data.response_byte = 0xFF;
      echo_data.timestamp = millis();
      echo_data.packet_count = packet_counter % 256;
      
      user.write((uint8_t*)&echo_data, sizeof(echo_data));
      
      USBSerial.printf("Stats sent via TCP: %lu packets processed\n", packet_counter);
    }
  }
}

void blink() {
  if (millis() - timer >= 200) {
    led = !led;
    digitalWrite(3, led);
    timer = millis();
  }
}

void check_tcp_connection() {
  static unsigned long last_reconnect = 0;
  
  if (tcp_enabled && !user.connected()) {
    USBSerial.println("TCP disconnected");
    tcp_enabled = false;
  }
  
  if (!tcp_enabled && pc_ip != IPAddress(0, 0, 0, 0) && (millis() - last_reconnect > 10000)) {
    USBSerial.println("TCP reconnection attempt...");
    tcp_connection_attempt();
    last_reconnect = millis();
  }
}

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, 1);
  setCpuFrequencyMhz(80);
  
  // Инициализация USB
  usb_init();
  delay(1000);
  
  USBSerial.println("\n╔═══════════════════════════════╗");
  USBSerial.println("║    ESP32 Echo v3 (TCP Fix)   ║");
  USBSerial.println("║    Auto-detect PC IP          ║");
  USBSerial.println("╚═══════════════════════════════╝");
  
  // Инициализация WiFi
  wifi_init();
  
  // Инициализация UDP
  udp_init();
  
  memset(&echo_data, 0, sizeof(EchoData));
  
  USBSerial.println("Setup complete!");
  USBSerial.println("📍 Send UDP packet to auto-detect PC IP for TCP");
  sending_mode = true;  // Начинаем в рабочем режиме
}

void loop() {
  // Мигание LED
  blink();
  
  // Обработка UDP эхо (основная функция)
  if (sending_mode) {
    handle_udp_echo();
  }
  
  // Обработка TCP команд
  handle_tcp_commands();
  
  // Периодическая отправка данных через TCP
  send_periodic_data();
  
  // Проверка TCP подключения
  check_tcp_connection();
}