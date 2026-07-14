/**
 * Topic A: /cmd_vel 1:1 Publisher
 * Code Only: QoS purely from rclcpp, no XML profile
 */
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cmd_vel_pub");

  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);

  auto pub = node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos);

  auto timer = node->create_wall_timer(
      100ms, [&pub]() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = 0.1;
        pub->publish(msg);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
