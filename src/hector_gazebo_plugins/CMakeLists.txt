cmake_minimum_required(VERSION 3.8)
project(hector_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)     # Find Gazebo libraries and includes
find_package(laser_interfaces REQUIRED)
# Include directories
# include_directories(include ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) # <-- 移除 Boost include
include_directories(include ${GAZEBO_INCLUDE_DIRS}) # <-- 只保留 Gazebo include
link_directories(${GAZEBO_LIBRARY_DIRS}) # Let linker know where to find Gazebo libs

# Build the shared library for the plugin
add_library(hector_gazebo_plugin SHARED src/hector_gazebo_plugin.cpp)

# Add dependencies for linking
ament_target_dependencies(hector_gazebo_plugin
        rclcpp
        gazebo_ros
        laser_interfaces
)

# Explicitly link Gazebo libraries needed by the plugin
# ament_target_dependencies should handle dependencies found via find_package,
# but explicitly linking GAZEBO_LIBRARIES is often safer for plugins.
target_link_libraries(hector_gazebo_plugin ${GAZEBO_LIBRARIES})

# Install the plugin library
install(TARGETS hector_gazebo_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin # 虽然插件是 .so，但也添加 runtime 以防万一
)

# Install package.xml
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME}
)

# Export Gazebo plugin path so Gazebo can find it
# Using install(TARGETS ...) handles exporting the library target correctly for CMAKE.
# The package.xml export tag <gazebo_ros gazebo_plugin_path="${prefix}/lib" /> is crucial for Gazebo runtime discovery.

# Install launch files if any (optional)
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Export dependencies needed by downstream packages that depend on this one
ament_export_dependencies(gazebo_ros rclcpp laser_interfaces)

# Final package processing
ament_package()