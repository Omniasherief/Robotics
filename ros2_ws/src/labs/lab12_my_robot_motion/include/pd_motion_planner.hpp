#ifndef PD_MOTION_PLANNER_HPP
#define PD_MOTION_PLANNER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace my_robot_motion
{
    class PDMotionPlanner : public rclcpp::Node
    {
    public:
        PDMotionPlanner();

    private:
        // ROS Interfaces
        rclcpp::TimerBase::SharedPtr control_loop_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // PD Control Parameters
        double kp_;
        double kd_;
        double step_size_;
        double max_linear_velocity_;
        
        // Logic Variables
        nav_msgs::msg::Path current_path_;
        bool path_received_ = false;
        double last_error_ = 0.0;

        // Callbacks
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void timer_callback();
    };
}

#endif