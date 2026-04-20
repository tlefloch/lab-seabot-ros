cmake_minimum_required(VERSION 3.8)
project(depth_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seabot_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(depth_filter
  src/depth_filter.cpp
  src/depth_filter_node.cpp
)

ament_target_dependencies(depth_filter rclcpp seabot_msgs std_msgs)

target_include_directories(depth_filter PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS
  depth_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
