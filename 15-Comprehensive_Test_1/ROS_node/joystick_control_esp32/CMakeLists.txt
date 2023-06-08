cmake_minimum_required(VERSION 3.5)
project(joystick_control_esp32)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joystick_node src/joy_control.cpp)
ament_target_dependencies(joystick_node rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS 
        joystick_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
