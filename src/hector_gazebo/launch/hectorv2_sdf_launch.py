import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# Removed: import xacro - no longer needed

robot_name_in_gazebo = 'hector_description'
initial_pose = [0.0, 0.0, 0.55, 0.0, 0.0, 0.0]

hector_gazebo_pkg_path = get_package_share_directory('hector_gazebo')

# --- Configure robot's SDF file ---
robot_description_subpath = 'robot.sdf'
sdf_file_path = os.path.join(hector_gazebo_pkg_path, 'assets', 'models', 'xacro', robot_description_subpath)

if not os.path.exists(sdf_file_path):
    raise FileNotFoundError(f"SDF file not found: {sdf_file_path}")
else:
    print(f"Using SDF file: {sdf_file_path}")

# --- Removed XACRO processing block ---
world_file_name = 'world.sdf'
world_path = os.path.join(hector_gazebo_pkg_path, 'assets', 'models', 'worlds', world_file_name)
if not os.path.exists(world_path):
    print(f"Warning: World file not found: {world_path}. Gazebo Sim will try to load default world.")
    gz_world_arg = ""
else:
    gz_world_arg = world_path


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # === run Gazebo Sim (Ignition) ===
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            # Pass the world file to Gazebo Sim
            'gz_args': f'-r -v 2 {gz_world_arg}'.strip()
        }.items()
    )

    # === Removed robot_state_publisher_node ===
    # Not typically used when spawning directly from SDF in Gazebo Sim.
    # TF will be handled by the bridge below.

    # === Spawn Robot from SDF file ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file_path,             # Use -file instead of -topic
            '-name', robot_name_in_gazebo,
            '-allow_renaming', 'true',
            # Initial pose arguments remain the same
            '-x', PythonExpression(f"str({initial_pose[0]})"),
            '-y', PythonExpression(f"str({initial_pose[1]})"),
            '-z', PythonExpression(f"str({initial_pose[2]})"),
            '-R', PythonExpression(f"str({initial_pose[3]})"),
            '-P', PythonExpression(f"str({initial_pose[4]})"),
            '-Y', PythonExpression(f"str({initial_pose[5]})")
        ],
        output='screen'
    )

    # === Bridge for TF transforms ===
    # Gets the pose of the robot links from Gazebo and publishes them to ROS 2's /tf
    # tf_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         # Bridge Gazebo poses to ROS TF messages
    #         # Assumes Gazebo version uses gz.msgs (like Fortress/Garden). Use ignition.msgs for older versions.
    #         '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    #         # You might need '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' as well for static transforms
    #         # if your robot has fixed joints that aren't published otherwise.
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # === Removed explicit IMU bridge ===
    # Assuming your custom hector_gazebo_fortress_plugin handles publishing IMU data to ROS
    # based on its SDF configuration and reading from an Ignition Transport topic.
    # If IMU data is missing in ROS, double-check your SDF configuration for both the
    # Gazebo IMU sensor and the hector_gazebo_fortress_plugin, or re-add/adjust a bridge if needed.
    # Example original bridge (commented out):
    # gazebo_imu_topic = f'/world/default/model/{robot_name_in_gazebo}/link/imu_link/sensor/imu_sensor/imu'
    # ros2_imu_topic = '/trunk_imu'
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         # Make sure message types match your Gazebo version (gz.msgs or ignition.msgs)
    #         f'{gazebo_imu_topic}@sensor_msgs/msg/Imu@gz.msgs.IMU'
    #     ],
    #     remappings=[
    #         (gazebo_imu_topic, ros2_imu_topic),
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        launch_gazebo,
        # robot_state_publisher_node, # Removed
        spawn_robot,
        # tf_bridge, # Added TF bridge
        # bridge, # Removed explicit IMU bridge

    ])