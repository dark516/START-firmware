#!/usr/bin/env python3
"""
Быстрый автоматический тест без ввода от пользователя
Запускается сразу, делает 5 тестов и завершается
"""
import socket
import time

ESP32_IP = "192.168.125.222"
TEST_PORT = 3333

def quick_test():
    print("⚡ Быстрый тест связи с ESP32")
    print(f"🎯 {ESP32_IP}:{TEST_PORT}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    try:
        sock.bind(('', 0))
        
        success = 0
        total = 5
        
        for i in range(total):
            test_byte = (i + 1) * 10  # 10, 20, 30, 40, 50
            
            try:
                # Отправляем
                sock.sendto(bytes([test_byte]), (ESP32_IP, TEST_PORT))
                
                # Ждем ответ
                data, addr = sock.recvfrom(1024)
                
                if len(data) == 1 and data[0] == test_byte:
                    print(f"✅ Тест {i+1}: OK (байт {test_byte})")
                    success += 1
                else:
                    print(f"❌ Тест {i+1}: FAIL (ожидался {test_byte}, получен {data.hex()})")
                    
            except socket.timeout:
                print(f"⏰ Тест {i+1}: TIMEOUT")
            except Exception as e:
                print(f"❌ Тест {i+1}: ERROR - {e}")
                
            time.sleep(0.5)
            
        # Результат
        print(f"\n📊 Результат: {success}/{total} успешно")
        
        if success == total:
            print("🎉 ОТЛИЧНО! Связь работает")
            return True
        elif success > 0:
            print("⚠️ Частичный успех")
            return False
        else:
            print("❌ Связь не работает")
            return False
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        return False
    finally:
        sock.close()

if __name__ == "__main__":
    quick_test()