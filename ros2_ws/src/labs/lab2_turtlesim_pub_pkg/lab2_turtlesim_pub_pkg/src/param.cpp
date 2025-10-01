#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
//500ms, 2s, 1min instead of std::chrono::milliseconds(500)

class Talker : public rclcpp::Node
{
    public:
    
    Talker() : Node("turtlesim_pub"), count_(0)
    {       
    this->declare_parameter("robot_speed", 0.4);    
    publisher_= this-> create_publisher <geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback,this));
    }
private:
void timer_callback()
{
   auto message = geometry_msgs::msg::Twist();
   // message.data ="hello, world"+ std::to_string(count_ ++);
   //  RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
   double my_param = this->get_parameter("robot_speed").as_double();
   message.linear.x=my_param;
   message.angular.z=my_param;

  RCLCPP_INFO(this->get_logger(), "publish : linear velocity in x direction = %f", message.linear.x);
  RCLCPP_DEBUG(this->get_logger(), "publish : linear velocity in x direction =  %f'", message.linear.x);
  RCLCPP_WARN(this->get_logger(), "publish : linear velocity in x direction = %f ", message.linear.x);


  RCLCPP_INFO(this->get_logger(), "puplish :  angular velocity in z direction = %f ",  message.angular.z);

   publisher_ -> publish(message);
   //std::vector<rclcpp::Parameter> robot_spped{rclcpp::Parameter("robot_speed", 0.4)};
   // this->set_parameters(robot_speed);

}
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
rclcpp::TimerBase::SharedPtr timer_;
int count_;
std::vector<rclcpp::Parameter> robot_spped{rclcpp::Parameter("robot_speed", 0.4)};//
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);// 1. Initialize ROS
    rclcpp::spin(std::make_shared<Talker>());// 2. Create node & keep it alive
    rclcpp::shutdown();// 3. Clean up
    return 0;
}