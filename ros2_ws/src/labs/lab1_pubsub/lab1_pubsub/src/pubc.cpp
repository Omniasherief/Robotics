#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
//500ms, 2s, 1min instead of std::chrono::milliseconds(500)

class Talker : public rclcpp::Node
{
    public:
    
    Talker() : Node("talker"), count_(0)
    {
    publisher_= this-> create_publisher <std_msgs::msg::String>("chatter",10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback,this));
    }
private:
void timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data ="hello, world"+ std::to_string(count_ ++);
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());
    publisher_ -> publish(message);

}
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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