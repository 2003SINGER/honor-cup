cmake_minimum_required(VERSION 3.8)
project(yahboom_laser_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


add_executable(laser_filter_node 
    src/laser_filter_node.cpp 
    src/laser_filter_processor.cpp)
ament_target_dependencies(laser_filter_node rclcpp sensor_msgs)

install(TARGETS laser_filter_node
        DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(
  DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME})

ament_package()


