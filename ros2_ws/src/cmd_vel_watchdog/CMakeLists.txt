cmake_minimum_required(VERSION 3.8)
project(cmd_vel_watchdog)

# Компилятор C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Исполняемый файл
add_executable(cmd_vel_watchdog_node src/cmd_vel_watchdog_node.cpp)

ament_target_dependencies(cmd_vel_watchdog_node
  rclcpp
  geometry_msgs
)

# Установка
install(TARGETS
  cmd_vel_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch файлов (если есть)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()