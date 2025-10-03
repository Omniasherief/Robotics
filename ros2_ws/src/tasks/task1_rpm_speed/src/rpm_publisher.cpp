#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class RpmPublisher:public rclcpp::Node
{
    public:
    RpmPublisher():Node("rpm_publisher")
    {
        publisher_=this->create_publisher<std_msgs::msg::Float64>("/rpm",10);
        timer_=this->create_wall_timer(std::chrono::seconds(1),std::bind(&RpmPublisher::publish_rpm,this));

    }
    private:
    void publish_rpm()
    {
        auto msg = std_msgs::msg::Float64();
        msg.data=60.0; //const rpm
        publisher_ ->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing RPM: %f", msg.data);

    }
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main (int argc , char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RpmPublisher>());
    rclcpp::shutdown();
    return 0;
}