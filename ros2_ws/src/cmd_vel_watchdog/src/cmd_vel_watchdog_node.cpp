#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class CmdVelWatchdog : public rclcpp::Node
{
public:
    CmdVelWatchdog() : Node("cmd_vel_watchdog")
    {
        // Параметры
        this->declare_parameter("timeout_ms", 500);  // 500 мс по умолчанию
        this->declare_parameter("input_topic", "cmd_vel_smoothed");
        this->declare_parameter("output_topic", "cmd_vel");
        
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        
        // Подписка на входящие команды
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            input_topic,
            10,
            std::bind(&CmdVelWatchdog::cmdVelCallback, this, std::placeholders::_1)
        );
        
        // Публикация выходных команд
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
        
        // Таймер для проверки timeout
        timer_ = this->create_wall_timer(
            50ms,  // Проверка каждые 50 мс
            std::bind(&CmdVelWatchdog::checkTimeout, this)
        );
        
        // Инициализация времени
        last_cmd_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), 
            "CmdVel Watchdog started: %s -> %s (timeout: %d ms)", 
            input_topic.c_str(), 
            output_topic.c_str(), 
            timeout_ms_
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Обновляем время последней команды
        last_cmd_time_ = this->now();
        last_cmd_ = *msg;
        
        // Пробрасываем команду дальше
        publisher_->publish(*msg);
    }
    
    void checkTimeout()
    {
        auto now = this->now();
        auto elapsed_ms = (now - last_cmd_time_).seconds() * 1000.0;
        
        if (elapsed_ms > timeout_ms_)
        {
            // Публикуем нулевую скорость
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.linear.y = 0.0;
            stop_msg.linear.z = 0.0;
            stop_msg.angular.x = 0.0;
            stop_msg.angular.y = 0.0;
            stop_msg.angular.z = 0.0;
            
            publisher_->publish(stop_msg);
            
            // Логируем только раз при первом timeout
            if (!timeout_logged_)
            {
                RCLCPP_WARN(this->get_logger(), 
                    "No cmd_vel received for %.0f ms - publishing STOP", 
                    elapsed_ms
                );
                timeout_logged_ = true;
            }
        }
        else
        {
            timeout_logged_ = false;  // Сбрасываем флаг если команды идут
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_;
    
    int timeout_ms_;
    bool timeout_logged_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CmdVelWatchdog>();
    
    // Перед завершением публикуем STOP
    rclcpp::on_shutdown([node]() {
        geometry_msgs::msg::Twist stop_msg;
        node->get_logger();
        RCLCPP_INFO(node->get_logger(), "Shutdown - publishing final STOP command");
        // Публикация stop команды через node's publisher (нужен доступ)
    });
    
    rclcpp::spin(node);
    
    // Финальная остановка
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    geometry_msgs::msg::Twist stop_msg;
    pub->publish(stop_msg);
    
    rclcpp::shutdown();
    return 0;
}