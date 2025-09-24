#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;
//500ms, 2s, 1min instead of std::chrono::milliseconds(500)

class Talker : public rclcpp::Node
{
    public:
    
    Talker() : Node("turtlesim_pub"), count_(0)
    {       
    publisher_= this-> create_publisher <geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback,this));
    }
private:
void timer_callback()
{
   auto message = geometry_msgs::msg::Twist();
   // message.data ="hello, world"+ std::to_string(count_ ++);
   //  RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
   message.linear.x=1.8;
   message.angular.z=0.6;

  RCLCPP_INFO(this->get_logger(), "publish : linear velocity in x direction = %f", message.linear.x);
  RCLCPP_DEBUG(this->get_logger(), "publish : linear velocity in x direction =  %f'", message.linear.x);
  RCLCPP_WARN(this->get_logger(), "publish : linear velocity in x direction = %f ", message.linear.x);


  RCLCPP_INFO(this->get_logger(), "puplish :  angular velocity in z direction = %f ",  message.angular.z);

   publisher_ -> publish(message);


}
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_;
int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);// 1. Initialize ROS
    rclcpp::spin(std::make_shared<Talker>());// 2. Create node & keep it alive
    rclcpp::shutdown();// 3. Clean up
    return 0;
}