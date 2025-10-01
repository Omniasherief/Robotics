#include "rclcpp/rclcpp.hpp"
#include "lab3_custom_msg_pkg/srv/sum.hpp"                                        // CHANGE

#include <memory>

void add(const std::shared_ptr<lab3_custom_msg_pkg::srv::Sum::Request> request,     // CHANGE
          std::shared_ptr<lab3_custom_msg_pkg::srv::Sum::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b ;                                      // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\n a: %ld" " b: %ld",  // CHANGE
                request->a, request->b);                                         // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

  rclcpp::Service<lab3_custom_msg_pkg::srv::Sum>::SharedPtr service =               // CHANGE
    node->create_service<lab3_custom_msg_pkg::srv::Sum>("add_2_ints",  &add);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add 2 ints.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}
