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
data_files_path = os.path.join(get_package_prefix(hector_logs_pkg)).replace('install', 'src/hector-ros2')
simulator_name = 'gazebo'                     

def generate_launch_description():

  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gui + gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Load custom world')

    # Start robot state publisher
  robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_raw,
                    'use_sim_time':True}])
  
    # Pause simulation at the beginning
  declare_pause_sim_cmd = DeclareLaunchArgument(
        name='pause',
        default_value='true',
        description='Whether to start Gazebo paused')
    
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

    # Load joint state controller
  load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
            'joint_state_broadcaster'],
        output='screen' )

    # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_path, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

    # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_path, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # Start RViz 
  rviz2_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'echo "\n/**************   Starting Hector RViz2   **************/\n" && '
                                                          'source /opt/ros/humble/setup.bash &&'
                                                          'ros2 run rviz2 rviz2 -d ' + rviz_config_file],
        output='screen'
    )
  odom_publisher=Node(
                      package='hector_assets',
                      executable='odom_publisher.py',
                      name='odom_publisher',
                      output='screen'
        )
  
  # Hector data logger
  data_logger_node = Node(
            package='hector_logs',
            executable='main_logger',
            name='main_logger',
            output='screen',
            parameters=[{'simulator_name': simulator_name}]             
        )
  
  # Hector data plotter
  plotter_node = Node(
            package='hector_logs',
            executable='gen_plots.py',
            name='plotter_node',
            output='screen',
            parameters=[{'simulator_name': simulator_name},
                        {'data_files_path': data_files_path},
                        {'time_steps': 5000} # Specify number of time steps to plot. 
                                             # If more please increase delay below.
                        ]             
        )

  # Delayed start to run experiments before plotting
  delayed_plot = TimerAction(
        period=15.0,  # Delay in seconds
        actions=[plotter_node]
    )  
  
  # Delayed start to give time for gazebo to load
  delayed_odom_publisher_start = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[odom_publisher]
    )  
  controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['hector_controller'],  
        output='screen'
    )  

  return  LaunchDescription([    
    RegisterEventHandler(
      event_handler=OnProcessExit(
        target_action=spawn_robot,
        on_exit=[load_joint_state_controller],
      )
    ),
    declare_simulator_cmd,
    declare_use_sim_time_cmd,
    declare_use_simulator_cmd,
    declare_world_cmd,
    declare_pause_sim_cmd,

    start_gazebo_server_cmd,
    start_gazebo_client_cmd,

    spawn_robot,
    robot_state_publisher_node,
    controller_spawner_node,
    # rviz2_node,
    # delayed_odom_publisher_start,
    # data_logger_node,
    # delayed_plot  
 ])