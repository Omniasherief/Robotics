#include "pd_motion_planner.hpp"

namespace my_robot_motion {

PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner") {
    // Declare parameters so you can change them in the launch file
    this->declare_parameter("kp", 0.5);
    this->declare_parameter("kd", 0.1);
    
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "plan", 10, std::bind(&PDMotionPlanner::path_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // 20Hz control loop
    control_loop_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&PDMotionPlanner::timer_callback, this));
}

void PDMotionPlanner::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = *msg;
    path_received_ = true;
}

void PDMotionPlanner::timer_callback() {
    if (!path_received_ || current_path_.poses.empty()) return;

    // 1. Get current robot pose via TF (map -> base_footprint)
    // 2. Calculate error to the next point in current_path_
    double current_error = 0.0; // Simplified placeholder
    
    // 3. PD Calculation
    double derivative = current_error - last_error_;
    double steering_output = (kp_ * current_error) + (kd_ * derivative);
    last_error_ = current_error;

    // 4. Publish Velocity
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.2; // Move forward at constant speed
    cmd.angular.z = steering_output;
    cmd_pub_->publish(cmd);
}
}