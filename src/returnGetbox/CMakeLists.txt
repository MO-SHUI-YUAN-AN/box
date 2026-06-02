cmake_minimum_required(VERSION 3.8)
project(return_getbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(grip_detect_node src/main.cpp)
target_link_libraries(grip_detect_node ${OpenCV_LIBS})
target_compile_features(grip_detect_node PUBLIC cxx_std_17)
ament_target_dependencies(grip_detect_node rclcpp robot_interfaces)

install(TARGETS grip_detect_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
