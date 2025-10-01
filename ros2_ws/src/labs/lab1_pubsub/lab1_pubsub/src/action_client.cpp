#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lab3_custom_msg_pkg/action/path_plan.hpp"
#include "geometry_msgs/msg/point.hpp"

typedef lab3_custom_msg_pkg::action::PathPlan NavigateAction;

class NavigateActionClientNode : public rclcpp::Node
{
public:
  NavigateActionClientNode(float x, float y, float z) 
  : Node("navigate_action_client_node")
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateAction>(this, "navigate");

    while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

   
    auto goal_msg = NavigateAction::Goal();
    goal_msg.goal_point.x = x;
    goal_msg.goal_point.y = y;
    goal_msg.goal_point.z = z;

    RCLCPP_INFO(this->get_logger(), "Sending goal (%.2f, %.2f, %.2f)", x, y, z);


    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateActionClientNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateActionClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&NavigateActionClientNode::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateAction>::SharedPtr client_ptr_;

void goal_response_callback(
    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateAction>> goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
  }
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
    const std::shared_ptr<const NavigateAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f", feedback->distance_to_point);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Elapsed time: %.2f seconds", result.result->elapsed_time);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Usage: ros2 run lab1_pubsub action_client x y z\n";
    return 1;
  }

  float x = std::stof(argv[1]);
  float y = std::stof(argv[2]);
  float z = std::stof(argv[3]);

  auto node = std::make_shared<NavigateActionClientNode>(x, y, z);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
