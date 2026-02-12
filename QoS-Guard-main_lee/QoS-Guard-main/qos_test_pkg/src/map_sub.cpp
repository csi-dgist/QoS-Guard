/**
 * Topic C: /map N:1 Subscriber
 * XML Entity Profile: map_subscriber_profile
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_sub");

  rclcpp::QoS qos(rclcpp::KeepLast(2));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos,
      [](const nav_msgs::msg::OccupancyGrid::SharedPtr) {});

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
