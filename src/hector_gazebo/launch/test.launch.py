"""
Author: Omar Kolt
Date: May 31, 2024
Contact: omarkolt@hotmail.com
Description: This launch file is used to start the Hector robot simulation in Gazebo,
             including necessary controllers and spawners.
"""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro
print("---------------------------------------------------------------------------------------------")
# Robot name
robot_name = 'hector_description'

# Set initial pose of the robot [x, y, z, roll, pitch, yaw]
initial_pose = [0.0, 0.0, 0.55, 0.0, 0.0, 0.0]

# configure robot's urdf file
hector_assets_pkg = 'hector_assets'
robot_description_subpath = 'models/xacro/robot.xacro'
xacro_file = os.path.join(get_package_share_directory(hector_assets_pkg),robot_description_subpath)
robot_description_raw = xacro.process_file(xacro_file).toxml()

#configure paths
gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
hector_assets_pkg_path = get_package_share_directory('hector_assets')

# Set the path to the world file
world_file_name = 'empty.world'
world_path = os.path.join(hector_assets_pkg_path, 'models' , 'worlds', world_file_name)
rviz_config_file = os.path.join(hector_assets_pkg_path, 'config', 'rviz', 'hectorv2.rviz')

# Hector logs package
hector_logs_pkg = 'hector_logs'
# data_files_path = os.path.join(get_package_prefix(hector_logs_pkg)).replace('install', 'src/hector-ros2')
simulator_name = 'gazebo'

def generate_launch_description():
    # Start robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                     'use_sim_time':True}])

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name,
            '-x', str(initial_pose[0]), '-y', str(initial_pose[1]), '-z', str(initial_pose[2]),
            '-R', str(initial_pose[3]), '-P', str(initial_pose[4]), '-Y', str(initial_pose[5])
        ],
        output='screen')



    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(
                'gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world',world_path),('verbose','true')]
    )

    # Load joint state controller
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'], # 将要加载并启动的控制器名称放在 arguments 列表里
        output='screen'
    )

    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hector_controller'],
        output='screen'
    )


    return  LaunchDescription([
        launch_gazebo,
        robot_state_publisher_node,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        controller_spawner_node,
    ])