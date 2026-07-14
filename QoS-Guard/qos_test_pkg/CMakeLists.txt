cmake_minimum_required(VERSION 3.8)
project(qos_test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Topic A: /cmd_vel 1:1
add_executable(cmd_vel_pub src/cmd_vel_pub.cpp)
ament_target_dependencies(cmd_vel_pub rclcpp geometry_msgs)

add_executable(cmd_vel_sub src/cmd_vel_sub.cpp)
ament_target_dependencies(cmd_vel_sub rclcpp geometry_msgs)

# Topic B: /sensor_data 1:N (1 pub, 3 subs)
add_executable(sensor_pub src/sensor_pub.cpp)
ament_target_dependencies(sensor_pub rclcpp sensor_msgs)

add_executable(sensor_sub src/sensor_sub.cpp)
ament_target_dependencies(sensor_sub rclcpp sensor_msgs)

# Topic C: /map N:1 (2 pubs, 1 sub)
add_executable(map_pub src/map_pub.cpp)
ament_target_dependencies(map_pub rclcpp nav_msgs)

add_executable(map_sub src/map_sub.cpp)
ament_target_dependencies(map_sub rclcpp nav_msgs)

# Topic D: /status isolated
add_executable(status_pub src/status_pub.cpp)
ament_target_dependencies(status_pub rclcpp std_msgs)

add_executable(status_sub src/status_sub.cpp)
ament_target_dependencies(status_sub rclcpp std_msgs)

install(TARGETS
  cmd_vel_pub cmd_vel_sub
  sensor_pub sensor_sub
  map_pub map_sub
  status_pub status_sub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  profiles
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
