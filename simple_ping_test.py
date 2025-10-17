#!/usr/bin/env python3
"""
Простой тест обмена одним байтом с ESP32
1. PC отправляет байт на ESP32
2. ESP32 отправляет байт обратно на PC
3. Проверяем успешность обмена
"""
import socket
import time
from datetime import datetime

ESP32_IP = "192.168.125.222"
TEST_PORT = 3333  # Порт для тестирования

def ping_pong_test():
    print("🏓 Простой тест обмена байтом с ESP32")
    print("=" * 50)
    print(f"ESP32 IP: {ESP32_IP}")
    print(f"Порт: {TEST_PORT}")
    
    # Создаем UDP сокет
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)  # Таймаут 2 секунды
    
    try:
        # Привязываемся к любому порту для получения ответов
        sock.bind(('', 0))  
        local_port = sock.getsockname()[1]
        print(f"Слушаем ответы на порту: {local_port}")
        
        print("\n🚀 Запуск теста...")
        
        success_count = 0
        total_tests = 10
        
        for i in range(total_tests):
            test_byte = (i + 1) % 256  # Тестовые байты 1-10
            
            print(f"\n📤 Тест {i+1}/{total_tests}: Отправляем байт {test_byte}")
            
            # Отправляем байт на ESP32
            try:
                sent = sock.sendto(bytes([test_byte]), (ESP32_IP, TEST_PORT))
                print(f"   ✅ Отправлено: {sent} байт")
                
                # Ждем ответ от ESP32
                try:
                    data, addr = sock.recvfrom(1024)
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    
                    if len(data) == 1:
                        received_byte = data[0]
                        print(f"   📥 [{timestamp}] Получен байт: {received_byte} от {addr[0]}:{addr[1]}")
                        
                        if received_byte == test_byte:
                            print(f"   ✅ УСПЕХ! Байт вернулся корректно")
                            success_count += 1
                        else:
                            print(f"   ❌ ОШИБКА! Ожидался {test_byte}, получен {received_byte}")
                    else:
                        print(f"   ❌ ОШИБКА! Получено {len(data)} байт, ожидался 1")
                        print(f"   Данные: {data.hex()}")
                        
                except socket.timeout:
                    print(f"   ⏰ ТАЙМАУТ! ESP32 не ответил")
                    
            except Exception as e:
                print(f"   ❌ Ошибка отправки: {e}")
            
            time.sleep(0.5)  # Пауза между тестами
            
        # Итоговая статистика
        print("\n" + "=" * 50)
        print("📊 РЕЗУЛЬТАТЫ ТЕСТА:")
        print(f"Успешных обменов: {success_count}/{total_tests}")
        
        if success_count == total_tests:
            print("🎉 ОТЛИЧНО! Все тесты прошли успешно")
            print("✅ Связь с ESP32 работает корректно")
        elif success_count > 0:
            print(f"⚠️ Частичный успех ({success_count*100/total_tests:.0f}%)")
            print("🔧 Возможны проблемы с сетью или ESP32")
        else:
            print("❌ ПРОВАЛ! Ни один тест не прошел")
            print("🚨 Проверьте:")
            print("  1. ESP32 включен и прошит")
            print("  2. ESP32 подключен к WiFi")
            print("  3. IP адрес ESP32 корректный")
            print("  4. Порт не заблокирован")
            
    except Exception as e:
        print(f"❌ Критическая ошибка: {e}")
    finally:
        sock.close()

def continuous_test():
    """Непрерывный тест для отладки"""
    print("\n🔄 НЕПРЕРЫВНЫЙ РЕЖИМ (Ctrl+C для остановки)")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', 0))
        
        counter = 0
        while True:
            counter += 1
            test_byte = counter % 256
            
            try:
                sock.sendto(bytes([test_byte]), (ESP32_IP, TEST_PORT))
                
                try:
                    data, addr = sock.recvfrom(1024)
                    if len(data) == 1 and data[0] == test_byte:
                        print(f"✅ #{counter}: OK")
                    else:
                        print(f"❌ #{counter}: FAIL - got {data.hex()}")
                except socket.timeout:
                    print(f"⏰ #{counter}: TIMEOUT")
                    
            except Exception as e:
                print(f"❌ #{counter}: ERROR - {e}")
                
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n⏹️ Остановлено пользователем")
    finally:
        sock.close()

def main():
    print("🔧 Выберите режим тестирования:")
    print("1. Однократный тест (10 попыток)")
    print("2. Непрерывный тест")
    
    try:
        choice = input("\nВведите номер (1 или 2): ").strip()
        
        if choice == "1":
            ping_pong_test()
        elif choice == "2": 
            continuous_test()
        else:
            print("Запускаем однократный тест по умолчанию...")
            ping_pong_test()
            
    except KeyboardInterrupt:
        print("\n👋 До свидания!")

if __name__ == "__main__":
    main()