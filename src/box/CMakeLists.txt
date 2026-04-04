cmake_minimum_required(VERSION 3.8)
project(box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # clangd

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

###########################
find_package(interfaces REQUIRED)
###########################

# find dependencies
# find_package(robot_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(sensor_msgs REQUIRED)

add_library(inference STATIC src/inference.cpp)
target_include_directories(inference PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(inference ${OpenCV_LIBS})

add_executable(main src/main.cpp)
target_include_directories(main PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

add_executable(tests src/tests.cpp)
target_link_libraries(tests ${OpenCV_LIBS} inference ${realsense2_LIBRARY})

target_link_libraries(main ${OpenCV_LIBS} inference ${realsense2_LIBRARY})

target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(main
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
)

ament_target_dependencies(
  main
  rclcpp
  sensor_msgs
  # robot_interfaces
  interfaces
)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
