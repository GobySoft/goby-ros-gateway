cmake_minimum_required(VERSION 3.8)
project(goby_ros_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GOBY 3.0 REQUIRED zeromq)
include_directories("${GOBY_INCLUDE_DIR}")

add_executable(goby_ros_gateway src/goby_ros_gateway.cpp)
ament_target_dependencies(goby_ros_gateway rclcpp std_msgs)
target_link_libraries(goby_ros_gateway goby goby_zeromq goby_moos)

install(TARGETS
  goby_ros_gateway
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
