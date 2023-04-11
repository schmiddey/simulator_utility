cmake_minimum_required(VERSION 3.8)
project(sim_joy2vel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies ament_cmake rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(sim_joy2vel_ps4_node src/SimJoy2Vel_ps4_node.cpp)
ament_target_dependencies(sim_joy2vel_ps4_node ${dependencies})

install(TARGETS sim_joy2vel_ps4_node
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY images
#   DESTINATION share/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
