cmake_minimum_required(VERSION 3.8)
project(goby_ros_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GOBY 3.0 REQUIRED zeromq)
find_package(Protobuf REQUIRED)

include_directories("${GOBY_INCLUDE_DIR}")
set(Protobuf_IMPORT_DIRS "${GOBY_INCLUDE_DIR}")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS include/goby_ros_gateway/goby_ros_gateway_config.proto)

configure_file(include/goby_ros_gateway/goby_ros_gateway.h ${CMAKE_CURRENT_BINARY_DIR}/include/goby_ros_gateway/goby_ros_gateway.h COPYONLY)

add_executable(goby_ros_gateway_app src/goby_ros_gateway.cpp)
set_target_properties(goby_ros_gateway_app PROPERTIES OUTPUT_NAME goby_ros_gateway)
ament_target_dependencies(goby_ros_gateway_app rclcpp)
target_link_libraries(goby_ros_gateway_app goby goby_zeromq protobuf::libprotobuf goby_ros_gateway)
target_include_directories(goby_ros_gateway_app
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
    $<INSTALL_INTERFACE:include>)
  
add_library(goby_ros_gateway SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(goby_ros_gateway "-rdynamic" goby goby_zeromq protobuf::libprotobuf)

install(TARGETS
  goby_ros_gateway
  EXPORT goby_ros_gateway_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

install(TARGETS
  goby_ros_gateway_app
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/goby_ros_gateway_config.pb.h
  DESTINATION include/goby_ros_gateway
)


ament_export_targets(goby_ros_gateway_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf GOBY)

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
