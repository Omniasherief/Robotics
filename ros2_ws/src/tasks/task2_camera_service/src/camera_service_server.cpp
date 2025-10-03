#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "task2_camera_service/srv/camera_angle.hpp"

class CameraServer : public rclcpp::Node
{
    public:
    CameraServer() :Node("camera_server")
    {
        service_ =this->create_service<task2_camera_service::srv::CameraAngle>("get_camera_image", std::bind(&CameraServer::handle_request , this, std::placeholders::_1 ,std::placeholders::_2));

    }
    private:
     
    void handle_request (  const std::shared_ptr<task2_camera_service::srv::CameraAngle::Request> request,
        std::shared_ptr<task2_camera_service::srv::CameraAngle::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received angle: %f", request->angle);

            std::string img_path;
        if (request->angle == -30.0)
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/-30.png";
        else if (request->angle == -15.0)
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/-15.png";
        else if (request->angle == 0.0)
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/0.png";
        else if (request->angle == 15.0)
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/15.png";
        else if (request->angle == 30.0)
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/30.png";
        else
        {
            RCLCPP_WARN(this->get_logger(), "No image for this angle!");
            img_path = "/home/omnia/ros2_ws/src/tasks/task2_camera_service/images/0.png"; // default
        }

        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);

        if (img.empty())
        {
             RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", img_path.c_str());
            img = cv::Mat::zeros(480, 640, CV_8UC3); // fallback
        }    
    response->image = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
}
  rclcpp::Service<task2_camera_service::srv::CameraAngle>::SharedPtr service_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraServer>());
    rclcpp::shutdown();
    return 0;
}
