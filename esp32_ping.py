#!/usr/bin/env python3
"""
Простой тест связи с ESP32
Проверяем базовую UDP связь
"""
import socket
import time

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333

def test_esp32_response():
    print("🔍 Тест связи с ESP32")
    print(f"IP: {ESP32_IP}, Порт: {CMD_PORT}")
    print("=" * 40)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    try:
        # Пробуем отправить разные команды
        test_commands = [
            "0.0 0.0",          # Стоп моторов
            "PING",             # Простой пинг  
            "STATUS",           # Запрос статуса
            bytes([0xFF]),      # Тестовый байт
            bytes([0xA5, 0x20]), # LIDAR команда
        ]
        
        for i, cmd in enumerate(test_commands, 1):
            print(f"\n🔄 Тест {i}/5: ", end="")
            
            if isinstance(cmd, str):
                data = cmd.encode('utf-8')
                print(f"'{cmd}'")
            else:
                data = cmd
                print(f"bytes {[hex(b) for b in cmd]}")
                
            try:
                # Отправляем
                sent = sock.sendto(data, (ESP32_IP, CMD_PORT))
                print(f"    📤 Отправлено: {sent} байт")
                
                # Пробуем получить ответ
                try:
                    response, addr = sock.recvfrom(1024)
                    print(f"    📥 Ответ от {addr}: {len(response)} байт")
                    
                    # Показываем содержимое
                    try:
                        text = response.decode('utf-8', errors='ignore').strip()
                        if text:
                            print(f"    📄 Текст: '{text}'")
                        else:
                            print(f"    📄 Hex: {response.hex()}")
                    except:
                        print(f"    📄 Hex: {response.hex()}")
                        
                except socket.timeout:
                    print(f"    ⏰ Нет ответа (таймаут 2с)")
                    
            except Exception as e:
                print(f"    ❌ Ошибка отправки: {e}")
            
            time.sleep(0.5)  # Пауза между тестами
            
    except Exception as e:
        print(f"❌ Общая ошибка: {e}")
    finally:
        sock.close()
    
    print("\n" + "=" * 40)
    print("💡 Если ESP32 не отвечает:")
    print("1. Проверьте питание и индикаторы")  
    print("2. Перезагрузите ESP32")
    print("3. Проверьте прошивку")
    print("4. Проверьте WiFi подключение")
    print("5. Проверьте IP адрес (возможно изменился)")

if __name__ == "__main__":
    test_esp32_response()