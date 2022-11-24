cmake_minimum_required(VERSION 3.10.2)

project(grasp_place_robot_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(behaviortree_cpp)

add_executable(${PROJECT_NAME} "bt_demo.cpp")
target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp)