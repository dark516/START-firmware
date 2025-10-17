#include <WiFi.h>
#include <WiFiUdp.h>

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

// TCP клиент для отправки данных (как в вашем коде)
const char* host = "192.168.125.71";  // IP компьютера
const int tcp_port = 8888;            // TCP порт для данных
WiFiClient user;

// Переменные
EchoData echo_data;
bool sending_mode = false;
unsigned long timer = 0, last_send = 0;
unsigned long packet_counter = 0;
bool led = false;

void usb_init() {

  Serial.begin(115200);
}

void wifi_init() {
  WiFi.disconnect();
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
}

void tcp_connection_initialize() {
  if (user.connect(host, tcp_port)) {
    Serial.printf("Connected to TCP server %s:%d\n", host, tcp_port);
    user.print("ESP32_ECHO_READY");
  } else {
    Serial.println("TCP connection failed (optional)");
  }
}

void udp_init() {
  if (udp.begin(UDP_PORT)) {
    Serial.printf("UDP server started on port %d\n", UDP_PORT);
  } else {
    Serial.println("UDP server failed to start");
  }
}

void handle_udp_echo() {
  int packetSize = udp.parsePacket();
  
  if (packetSize > 0) {
    // Читаем входящий байт
    uint8_t incomingByte;
    int bytesRead = udp.read(&incomingByte, 1);
    
    if (bytesRead == 1) {
      packet_counter++;
      
      // Получаем IP и порт отправителя
      IPAddress senderIP = udp.remoteIP();
      uint16_t senderPort = udp.remotePort();
      
      // Отправляем эхо обратно через UDP
      udp.beginPacket(senderIP, senderPort);
      udp.write(&incomingByte, 1);
      
      if (udp.endPacket()) {
        // Успешно отправлено
        if (packet_counter % 10 == 0) {
          Serial.printf("Echo #%lu: byte %d -> %s:%d\n", 
                          packet_counter, incomingByte, 
                          senderIP.toString().c_str(), senderPort);
        }
        
        // Также отправляем статистику через TCP (как в вашем коде)
        if (user.connected()) {
          echo_data.received_byte = incomingByte;
          echo_data.response_byte = incomingByte;
          echo_data.timestamp = millis();
          echo_data.packet_count = packet_counter % 256;
          
          user.write((uint8_t*)&echo_data, sizeof(echo_data));
        }
        
      } else {
        Serial.printf("Failed to send echo for byte %d\n", incomingByte);
      }
    }
  }
}

void handle_tcp_commands() {
  if (user.connected() && user.available()) {
    String task = user.readStringUntil('\n');
    
    if (task == "die") {
      Serial.println("Restarting...");
      ESP.restart();
    } else if (task == "work") {
      sending_mode = true;
      Serial.println("Work mode - UDP echo active");
    } else if (task == "rest") {
      sending_mode = false;
      Serial.println("Rest mode - UDP echo paused");
    } else if (task == "stats") {
      Serial.printf("Packets processed: %lu\n", packet_counter);
    }
  }
}

void send_periodic_data() {
  if (sending_mode && last_send + 1000 < millis()) {
    last_send = millis();
    
    if (user.connected()) {
      // Отправляем периодическую статистику
      echo_data.received_byte = 0xFF;  // Маркер статистики
      echo_data.response_byte = 0xFF;
      echo_data.timestamp = millis();
      echo_data.packet_count = packet_counter % 256;
      
      user.write((uint8_t*)&echo_data, sizeof(echo_data));
      
      Serial.printf("Stats sent: %lu packets processed\n", packet_counter);
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

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, 1);
  setCpuFrequencyMhz(80);
  
  // Инициализация USB
  usb_init();
  delay(1000);
  
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.println("║    ESP32 Simple Echo v2       ║");
  Serial.println("║     UDP + TCP like yours      ║");
  Serial.println("╚═══════════════════════════════╝");
  
  // Инициализация WiFi
  wifi_init();
  
  // Инициализация UDP
  udp_init();
  
  // Попытка подключения к TCP серверу (опционально)
  tcp_connection_initialize();
  
  memset(&echo_data, 0, sizeof(EchoData));
  
  Serial.println("Setup complete! Ready for UDP echo test");
  sending_mode = true;  // Начинаем в рабочем режиме
}

void loop() {
  // Мигание LED (как в вашем коде)
  blink();
  
  // Обработка UDP эхо (основная функция)
  if (sending_mode) {
    handle_udp_echo();
  }
  
  // Обработка TCP команд (как в вашем коде)
  handle_tcp_commands();
  
  // Периодическая отправка данных через TCP
  send_periodic_data();
  
  // Проверка TCP подключения
  if (!user.connected()) {
    // Пытаемся переподключиться каждые 10 секунд
    static unsigned long last_reconnect = 0;
    if (millis() - last_reconnect > 10000) {
      Serial.println("TCP reconnecting...");
      tcp_connection_initialize();
      last_reconnect = millis();
    }
  }
}