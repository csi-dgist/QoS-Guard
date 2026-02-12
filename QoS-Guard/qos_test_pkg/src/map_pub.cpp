/**
 * Topic C: /map N:1 Publisher (2 Pub : 1 Sub)
 * publisher_id: 1 or 2, different QoS per publisher
 */
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int pub_id = 1;
  if (argc >= 2) {
    pub_id = std::atoi(argv[1]);
  }
  std::string node_name = "map_pub_" + std::to_string(pub_id);
  auto node = std::make_shared<rclcpp::Node>(node_name);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", qos);

  auto timer = node->create_wall_timer(
      1000ms, [&pub]() {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = "map";
        msg.info.width = 100;
        msg.info.height = 100;
        msg.info.resolution = 0.05f;
        msg.data.resize(10000, 0);
        pub->publish(msg);
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
