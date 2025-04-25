cmake_minimum_required(VERSION 3.5)
project(hector_control)

set(CMAKE_CXX_STANDARD 14)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_interfaces REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIR}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}  # Include sensor_msgs headers
)

add_subdirectory(third_party/qpOASES)

file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.h"
"ConvexMPC/*.cpp"
"ArmControl/*.cpp"
"ArmControl/*.h"
)

add_executable(hector_ctrl src/main.cpp ${SRC_LIST})

ament_target_dependencies(hector_ctrl
    rclcpp
    Boost
    laser_interfaces
    sensor_msgs
)

target_link_libraries(hector_ctrl
    qpOASES
    -pthread
)

install(TARGETS hector_ctrl
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_package()

