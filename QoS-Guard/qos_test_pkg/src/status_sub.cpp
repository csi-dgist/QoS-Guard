/**
 * Topic D: /status Isolated Subscriber
 * No matching publisher - no subscription target
 * Mixed Priority: has conflicting values between code and XML
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("status_sub");

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/status", qos,
      [](const std_msgs::msg::String::SharedPtr) {});

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
