/**
 * Topic B: /sensor_data 1:N Subscriber (profile index 1, 2, or 3)
 * Each instance uses different QoS via profile: sensor_sub_profile_1/2/3
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int profile_idx = 1;
  if (argc >= 2) {
    profile_idx = std::atoi(argv[1]);
  }
  std::string node_name = "sensor_sub_" + std::to_string(profile_idx);
  auto node = std::make_shared<rclcpp::Node>(node_name);

  rclcpp::QoS qos(rclcpp::KeepLast(5));
  if (profile_idx == 1) {
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  } else if (profile_idx == 2) {
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  } else {
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  }

  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/sensor_data", qos,
      [](const sensor_msgs::msg::LaserScan::SharedPtr) {});

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
