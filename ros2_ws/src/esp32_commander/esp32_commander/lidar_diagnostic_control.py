#!/usr/bin/env python3
"""
Управление диагностикой LIDAR на ESP32
"""
import rclpy
from rclpy.node import Node
import socket
import time

class LidarDiagnosticControl(Node):
    def __init__(self):
        super().__init__('lidar_diagnostic_control')
        
        # Параметры
        self.declare_parameter('esp32_ip', '192.168.125.222')
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.cmd_port = 3333
        
        # UDP сокет
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.get_logger().info("✅ LIDAR Diagnostic Control создан")
        self.get_logger().info(f"   ESP32: {self.esp32_ip}:{self.cmd_port}")
        self.get_logger().info("")
        self.get_logger().info("📋 ДОСТУПНЫЕ КОМАНДЫ:")
        self.get_logger().info("   ros2 service call /next_config std_srvs/srv/Empty")
        self.get_logger().info("   ros2 service call /retest std_srvs/srv/Empty") 
        self.get_logger().info("   ros2 service call /status std_srvs/srv/Empty")
        self.get_logger().info("")
        self.get_logger().info("🎯 ИЛИ используйте методы:")
        
        # Создаем сервисы
        from std_srvs.srv import Empty
        self.create_service(Empty, 'next_config', self.next_config_callback)
        self.create_service(Empty, 'retest', self.retest_callback)
        self.create_service(Empty, 'status', self.status_callback)
        
        # Таймеры для автоматического управления
        self.create_timer(2.0, self.send_initial_registration)
        self.auto_test_timer = None
        
        self.get_logger().info("🔴 Diagnostic Control готов!")
    
    def send_command(self, command):
        """Отправка команды на ESP32"""
        try:
            self.cmd_sock.sendto(command.encode(), (self.esp32_ip, self.cmd_port))
            self.get_logger().info(f"📤 Отправлена команда: '{command}'")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка отправки команды: {e}")
            return False
    
    def send_initial_registration(self):
        """Начальная регистрация у ESP32"""
        self.send_command("REGISTER")
        # Отменяем этот таймер после первого вызова
        if hasattr(self, 'initial_timer'):
            self.initial_timer.cancel()
    
    def next_config_callback(self, request, response):
        """Переключение на следующую конфигурацию LIDAR"""
        self.get_logger().info("🔄 Переключение на следующую конфигурацию LIDAR...")
        success = self.send_command("NEXT_CONFIG")
        if success:
            self.get_logger().info("✅ Команда NEXT_CONFIG отправлена")
        return response
    
    def retest_callback(self, request, response):
        """Повторное тестирование текущей конфигурации"""
        self.get_logger().info("🔄 Повторное тестирование текущей конфигурации...")
        success = self.send_command("RETEST")
        if success:
            self.get_logger().info("✅ Команда RETEST отправлена")
        return response
    
    def status_callback(self, request, response):
        """Запрос статуса текущей конфигурации"""
        self.get_logger().info("📊 Запрос статуса...")
        success = self.send_command("STATUS")
        if success:
            self.get_logger().info("✅ Команда STATUS отправлена")
        return response
    
    def start_auto_test(self):
        """Автоматическое тестирование всех конфигураций"""
        self.get_logger().info("🤖 Запуск автоматического тестирования...")
        self.get_logger().info("   Будут протестированы все конфигурации с интервалом 30 сек")
        
        # Таймер для автоматического переключения конфигураций
        self.auto_test_timer = self.create_timer(30.0, self.auto_next_config)
    
    def auto_next_config(self):
        """Автоматическое переключение конфигураций"""
        self.get_logger().info("🔄 Автоматическое переключение на следующую конфигурацию...")
        self.send_command("NEXT_CONFIG")
    
    def stop_auto_test(self):
        """Остановка автоматического тестирования"""
        if self.auto_test_timer:
            self.auto_test_timer.cancel()
            self.auto_test_timer = None
            self.get_logger().info("⏹️ Автоматическое тестирование остановлено")
    
    def print_help(self):
        """Печать справки"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║           LIDAR DIAGNOSTIC           ║")
        self.get_logger().info("║             СПРАВКА                  ║")
        self.get_logger().info("╠══════════════════════════════════════╣")
        self.get_logger().info("║ Команды ROS2 сервисов:              ║")
        self.get_logger().info("║                                      ║")
        self.get_logger().info("║ ros2 service call /next_config \\    ║")
        self.get_logger().info("║   std_srvs/srv/Empty                 ║")
        self.get_logger().info("║                                      ║")
        self.get_logger().info("║ ros2 service call /retest \\         ║")
        self.get_logger().info("║   std_srvs/srv/Empty                 ║")
        self.get_logger().info("║                                      ║")
        self.get_logger().info("║ ros2 service call /status \\         ║")
        self.get_logger().info("║   std_srvs/srv/Empty                 ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.get_logger().info("")

def main(args=None):
    rclpy.init(args=args)
    node = LidarDiagnosticControl()
    
    # Печать справки при запуске
    node.print_help()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Завершение работы...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()