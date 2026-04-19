cmake_minimum_required(VERSION 3.8)
project(simulation)

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(seabot_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  include_directories(
    /home/libs/eigen-5.0.0
  )

  add_executable(simulation
    src/simulation.cpp
    src/simulation_node.cpp
  )

  ament_target_dependencies(simulation rclcpp seabot_msgs std_msgs geometry_msgs)

  target_include_directories(simulation PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  )

  install(TARGETS
  simulation
  DESTINATION lib/${PROJECT_NAME})

  ament_package()
endif()
