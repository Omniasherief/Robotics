//Subscriber+Publisher /speed
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SpeedCalculator:public rclcpp::Node
{
public:
SpeedCalculator():Node("speed_calculator")
{
    this->declare_parameter<double>("wheel_radius",0.15);
    subscription_=this->create_subscription<std_msgs::msg::Float64>("/rpm",10,std::bind(&SpeedCalculator::rpm_callback , this ,std::placeholders::_1));
    publisher_= this->create_publisher<std_msgs::msg::Float64>("/speed",10);
}
private:
void rpm_callback (const std_msgs::msg::Float64::SharedPtr msg)
{
double rpm = msg->data;
double radius = this->get_parameter("wheel_radius").as_double();
double wheel_circumference =2 *M_PI * radius;
double speed = rpm * wheel_circumference /60.0; //m/s

auto out_msg=std_msgs::msg::Float64();
out_msg.data=speed;
publisher_ -> publish(out_msg);

RCLCPP_INFO(this-> get_logger(), "RPM: %.2f ->Speed: %.2f m/s",rpm,speed);
}
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_; 
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<SpeedCalculator>()); 
    rclcpp::shutdown(); 
    return 0; 
}