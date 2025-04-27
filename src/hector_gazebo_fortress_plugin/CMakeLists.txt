cmake_minimum_required(VERSION 3.16)
project(hector_gazebo_fortress_plugin)

# --- Set C++ Standard ---
# Gazebo Fortress and ROS 2 Humble/Iron typically use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_interfaces REQUIRED)

# Find Ignition Gazebo (Fortress) components
# Note: For Fortress, the package names are ignition-*.
#       For Garden and later, they are typically gz-*.
find_package(ignition-gazebo6 REQUIRED) # Corresponds to Fortress
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-math6 REQUIRED) # Common dependency
# Add other ignition common libraries if needed, e.g., ignition-common4

# --- Build the Plugin Library ---
add_library(${PROJECT_NAME} SHARED
        src/hector_gazebo_fortress_plugin.cc
)

# --- Link Dependencies ---

# 1. Link ROS 2 dependencies using ament helper
ament_target_dependencies(${PROJECT_NAME} PUBLIC # Use PUBLIC if headers include ROS headers
        rclcpp
        laser_interfaces
)

# 2. Link Ignition/Gazebo dependencies
# Use the NAMESPACE::TargetName format provided by find_package
target_link_libraries(${PROJECT_NAME} PUBLIC
        ignition-gazebo6::ignition-gazebo6
        ignition-plugin1::ignition-plugin1
        ignition-transport11::ignition-transport11
        ignition-msgs8::ignition-msgs8
        ignition-math6::ignition-math6
)

# --- Include Directories ---
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# --- Installation ---
# Install shared library (Plugin)
# Destination 'lib' is standard for Gazebo plugins
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
)

# Install the local header files
# This installs the contents of 'include/' into '<prefix>/include/'
# e.g., include/my_plugin/header.hh -> <prefix>/include/my_plugin/header.hh
install(DIRECTORY include/
        DESTINATION include
)

# Install package.xml
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
)

# --- Export for Downstream Packages (ament) ---
# Export the include directory for downstream packages.
ament_export_include_directories(include)

# Export the library target itself.
ament_export_libraries(${PROJECT_NAME})

# Export dependencies needed by downstream packages that use this package.
# This should include direct dependencies.
ament_export_dependencies(
        # Ignition dependencies (runtime/link time)
        ignition-gazebo6
        ignition-plugin1
        ignition-transport11
        ignition-msgs8
        ignition-math6
        # ROS 2 dependencies
        rclcpp
        laser_interfaces
)

# --- Final Package Processing ---
ament_package()