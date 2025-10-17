#!/usr/bin/env python3
"""
TCP сервер для приема данных от ESP32 через user.write
Работает совместно с UDP тестом
"""
import socket
import threading
import struct
import time
from datetime import datetime

TCP_PORT = 8888
UDP_TEST_PORT = 3333

class ESP32Server:
    def __init__(self):
        self.clients = {}
        self.running = True
        
    def handle_client(self, client_socket, addr):
        """Обработка подключения ESP32"""
        print(f"🔗 ESP32 подключился: {addr[0]}:{addr[1]}")
        
        try:
            # Читаем начальное сообщение
            init_data = client_socket.recv(1024)
            if init_data:
                init_msg = init_data.decode('utf-8', errors='ignore')
                print(f"📋 Инициализация: '{init_msg}'")
            
            packet_count = 0
            
            while self.running:
                try:
                    # Читаем бинарную структуру EchoData (8 байт)
                    data = client_socket.recv(8)
                    
                    if len(data) == 8:
                        # Разбираем структуру: uint8, uint8, uint32, uint8
                        received_byte, response_byte, timestamp, packet_id = struct.unpack('<BBIB', data)
                        
                        packet_count += 1
                        
                        if packet_count <= 5 or packet_count % 10 == 0:
                            current_time = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            
                            if received_byte == 0xFF and response_byte == 0xFF:
                                # Статистическое сообщение
                                print(f"📊 [{current_time}] Stats: пакет #{packet_id}, время {timestamp}")
                            else:
                                # Эхо данные
                                print(f"📦 [{current_time}] Echo: байт {received_byte}→{response_byte}, пакет #{packet_id}")
                        elif packet_count == 6:
                            print("📦 ... (продолжается прием)")
                            
                    elif len(data) == 0:
                        print(f"🔌 ESP32 {addr[0]} отключился")
                        break
                    else:
                        print(f"⚠️ Неожиданный размер данных: {len(data)} байт")
                        
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    print(f"🔌 ESP32 {addr[0]} сбросил соединение")
                    break
                    
        except Exception as e:
            print(f"❌ Ошибка с клиентом {addr[0]}: {e}")
        finally:
            client_socket.close()
            if addr[0] in self.clients:
                del self.clients[addr[0]]
            print(f"🔌 Клиент {addr[0]} отключен (всего пакетов: {packet_count})")
    
    def send_command(self, esp32_ip, command):
        """Отправка команды ESP32"""
        if esp32_ip in self.clients:
            try:
                self.clients[esp32_ip].send(f"{command}\n".encode())
                print(f"📤 Команда '{command}' отправлена {esp32_ip}")
                return True
            except:
                print(f"❌ Ошибка отправки команды {esp32_ip}")
                return False
        else:
            print(f"⚠️ ESP32 {esp32_ip} не подключен")
            return False
    
    def start_server(self):
        """Запуск TCP сервера"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(1.0)
        
        try:
            server_socket.bind(('', TCP_PORT))
            server_socket.listen(5)
            print(f"🔴 TCP сервер запущен на порту {TCP_PORT}")
            print("🔍 Ожидание подключений ESP32...")
            
            while self.running:
                try:
                    client_socket, addr = server_socket.accept()
                    client_socket.settimeout(5.0)
                    
                    # Сохраняем клиента
                    self.clients[addr[0]] = client_socket
                    
                    # Запускаем обработчик в отдельном потоке
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket, addr)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except socket.timeout:
                    continue
                except OSError:
                    break
                    
        except Exception as e:
            print(f"❌ Ошибка TCP сервера: {e}")
        finally:
            server_socket.close()
            print("🔴 TCP сервер остановлен")
    
    def stop(self):
        """Остановка сервера"""
        self.running = False
        for client in self.clients.values():
            try:
                client.close()
            except:
                pass

def udp_ping_test(esp32_ip):
    """UDP тест пинга (из быстрого теста)"""
    print(f"\n⚡ UDP тест связи с {esp32_ip}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    
    try:
        sock.bind(('', 0))
        
        success = 0
        total = 5
        
        for i in range(total):
            test_byte = (i + 1) * 10
            
            try:
                sock.sendto(bytes([test_byte]), (esp32_ip, UDP_TEST_PORT))
                data, addr = sock.recvfrom(1024)
                
                if len(data) == 1 and data[0] == test_byte:
                    print(f"✅ UDP тест {i+1}: OK (байт {test_byte})")
                    success += 1
                else:
                    print(f"❌ UDP тест {i+1}: FAIL")
                    
            except socket.timeout:
                print(f"⏰ UDP тест {i+1}: TIMEOUT")
            except Exception as e:
                print(f"❌ UDP тест {i+1}: ERROR - {e}")
                
            time.sleep(0.5)
            
        print(f"📊 UDP результат: {success}/{total} успешно")
        return success > 0
        
    finally:
        sock.close()

def main():
    print("🚀 ESP32 TCP+UDP Test Server")
    print("=" * 50)
    
    server = ESP32Server()
    
    # Запускаем TCP сервер в отдельном потоке
    server_thread = threading.Thread(target=server.start_server)
    server_thread.daemon = True
    server_thread.start()
    
    print("\n💡 Команды:")
    print("  udp <ip> - UDP тест")
    print("  work <ip> - Включить режим работы")
    print("  rest <ip> - Отключить режим работы") 
    print("  stats <ip> - Запросить статистику")
    print("  quit - Выход")
    
    try:
        while True:
            cmd = input("\n> ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd.startswith("udp "):
                ip = cmd[4:].strip()
                udp_ping_test(ip)
            elif cmd.startswith("work "):
                ip = cmd[5:].strip()
                server.send_command(ip, "work")
            elif cmd.startswith("rest "):
                ip = cmd[5:].strip()
                server.send_command(ip, "rest")
            elif cmd.startswith("stats "):
                ip = cmd[6:].strip()
                server.send_command(ip, "stats")
            elif cmd == "help":
                print("udp 192.168.125.222 - пример UDP теста")
                print("work 192.168.125.222 - пример включения")
            else:
                print("❓ Неизвестная команда. Наберите 'help'")
                
    except KeyboardInterrupt:
        print("\n⏹️ Остановка...")
    finally:
        server.stop()

if __name__ == "__main__":
    main()