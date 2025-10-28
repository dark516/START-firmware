# src/lidar_parser/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(lidar_parser)

# ── Компиляторские флаги ──────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Зависимости ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(std_msgs      REQUIRED)

# ── Узлы (executables) ────────────────────────────────────────────────
add_executable(rplidar_tcp_node src/rplidar_tcp_node.cpp)
ament_target_dependencies(rplidar_tcp_node
  rclcpp sensor_msgs std_msgs)

add_executable(rplidar_tcp_node_upd src/rplidar_tcp_node_upd.cpp)
ament_target_dependencies(rplidar_tcp_node_upd
  rclcpp sensor_msgs std_msgs)

# ── Установка ─────────────────────────────────────────────────────────
install(TARGETS
  rplidar_tcp_node
  rplidar_tcp_node_upd
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()