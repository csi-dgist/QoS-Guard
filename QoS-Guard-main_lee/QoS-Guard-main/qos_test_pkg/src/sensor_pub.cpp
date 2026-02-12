/**
 * Topic B: /sensor_data 1:N Publisher (1 Pub : 3 Sub)
 * XML Topic Profile: /sensor_data topic profile takes precedence
 */
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sensor_pub");

  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "/sensor_data", qos);

  auto timer = node->create_wall_timer(
      50ms, [&pub]() {
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "laser";
        msg.angle_min = -1.57f;
        msg.angle_max = 1.57f;
        msg.angle_increment = 0.01f;
        msg.range_min = 0.1f;
        msg.range_max = 10.0f;
        msg.ranges.resize(314, 1.0f);
        pub->publish(msg);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
