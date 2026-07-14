/**
 * Topic D: /status Isolated Publisher
 * No matching subscriber - no connection target
 * Default Trap: uses is_default_profile="true"
 */
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("status_pub");

  rclcpp::QoS qos(rclcpp::KeepLast(5));
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  auto pub = node->create_publisher<std_msgs::msg::String>(
      "/status", qos);

  auto timer = node->create_wall_timer(
      500ms, [&pub]() {
        std_msgs::msg::String msg;
        msg.data = "status";
        pub->publish(msg);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
