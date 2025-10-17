#!/usr/bin/env python3
"""
Проверка текущего состояния ESP32
Определяем какая прошивка сейчас запущена
"""
import socket
import time

ESP32_IP = "192.168.125.222"

def test_port(port, name, test_data):
    """Тест одного порта"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', 0))
        
        # Отправляем тестовые данные
        sock.sendto(test_data, (ESP32_IP, port))
        
        # Пробуем получить ответ
        try:
            data, addr = sock.recvfrom(1024)
            return f"✅ {name}: Ответ {len(data)} байт - {data[:20].hex()}"
        except socket.timeout:
            return f"⏰ {name}: Нет ответа"
            
    except Exception as e:
        return f"❌ {name}: Ошибка - {e}"
    finally:
        sock.close()

def check_esp32():
    print("🔍 Проверка состояния ESP32")
    print(f"🎯 IP: {ESP32_IP}")
    print("=" * 40)
    
    # Тестируем разные порты и команды
    tests = [
        (3333, "UDP Echo", b"\x42"),                    # Тест эхо
        (3333, "Motor Cmd", b"0.0 0.0"),               # Команда моторов
        (3333, "LIDAR Cmd", bytes([0xA5, 0x20])),      # LIDAR команда
        (3335, "Sensor Port", b"test"),                # Порт датчиков
        (3334, "LIDAR Port", b"test"),                 # LIDAR порт
    ]
    
    results = []
    for port, name, data in tests:
        result = test_port(port, name, data)
        results.append(result)
        print(result)
        time.sleep(0.5)
    
    print("\n" + "=" * 40)
    print("💡 ДИАГНОЗ:")
    
    # Анализ результатов
    has_responses = any("✅" in result for result in results)
    
    if has_responses:
        echo_works = any("UDP Echo" in result and "✅" in result for result in results)
        
        if echo_works:
            print("🎉 ESP32 работает с эхо-сервером!")
            print("✅ Простая прошивка загружена корректно")
        else:
            print("⚠️ ESP32 отвечает, но не на эхо-команды")
            print("🔧 Возможно загружена другая прошивка")
    else:
        print("❌ ESP32 не отвечает на UDP команды")
        print("🚨 Необходимо:")
        print("  1. Проверить Serial Monitor")
        print("  2. Прошить ESP32 эхо-сервером")
        print("  3. Перезагрузить ESP32")

if __name__ == "__main__":
    check_esp32()