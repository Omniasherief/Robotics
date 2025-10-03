#include "rclcpp/rclcpp.hpp"
#include "task2_camera_service/srv/camera_angle.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"

using namespace std::chrono_literals;

class CameraClient : public rclcpp::Node
{
public:
    CameraClient(double angle) : Node("camera_client"), angle_(angle)
    {
        client_ = this->create_client<task2_camera_service::srv::CameraAngle>("get_camera_image");

        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        auto request = std::make_shared<task2_camera_service::srv::CameraAngle::Request>();
        request->angle = angle_;

        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto img_msg = result.get()->image;
            cv::Mat img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
            cv::imshow("Camera Image at angle " + std::to_string(angle_), img);
            cv::waitKey(0);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service");
        }
    }

private:
    double angle_;
    rclcpp::Client<task2_camera_service::srv::CameraAngle>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Usage: ros2 run task2_camera_service camera_client <angle>" << std::endl;
        return 1;
    }

    double angle = std::stod(argv[1]);  
    std::make_shared<CameraClient>(angle);

    rclcpp::shutdown();
    return 0;
}
