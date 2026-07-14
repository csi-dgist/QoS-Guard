/**
 * Topic A: /cmd_vel 1:1 Subscriber
 * XML Entity Profile: uses cmd_vel_subscriber_profile from profiles
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cmd_vel_sub");

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile);

  auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos,
      [](const geometry_msgs::msg::Twist::SharedPtr msg) {
        (void)msg;
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
