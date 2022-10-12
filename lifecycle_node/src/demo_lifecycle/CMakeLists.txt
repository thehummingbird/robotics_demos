cmake_minimum_required(VERSION 3.8)
project(demo_lifecycle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#demos
add_executable(listener 
  src/listener_node.cpp)
ament_target_dependencies(listener
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
)


add_executable(service_client 
  src/service_client.cpp)
ament_target_dependencies(service_client
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
)

add_executable(lifecycle_talker 
  src/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
)

install(TARGETS
  listener
  service_client
  lifecycle_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
