#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lab3_custom_msg_pkg/msg/my_new_sensor.hpp"                                            // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<lab3_custom_msg_pkg::msg::MyNewSensor>("my_sensor_topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = lab3_custom_msg_pkg::msg::MyNewSensor();                                   // CHANGE
    message.sesnor_id  = this->count_++;    
    message.temp = this->count_++;   
    message.air_pressure = this->count_++;  
    message.air_speed = this->count_++;              
    message.center.x=5   ;            // CHANGE
    message.center.y=3    ;    
    message.center.z=8 ;    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sensor id: '" << message.sesnor_id << "'");    // CHANGE
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing temp : '" << message.temp << "'");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing pressure: '" << message.air_pressure << "'");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing speed : '" << message.air_speed << "'");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing pose x: '" << message.center.x<< "'");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing pose y : '" << message.center.y<< "'");
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing pose z : '" << message.center.z<< "'");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<lab3_custom_msg_pkg::msg::MyNewSensor>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}