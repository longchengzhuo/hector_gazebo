import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


robot_name_in_gazebo = 'hector_description' 
initial_pose = [0.0, 0.0, 0.7, 0, -1.57, 0.0]

hector_gazebo_pkg_path = get_package_share_directory('hector_gazebo')

# Configure robot's URDF/XACRO file
robot_description_subpath = 'robot.xacro'
xacro_file = os.path.join(hector_gazebo_pkg_path, 'assets', 'models', 'xacro', robot_description_subpath)
try:
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    print("XACRO processing successful.")
except Exception as e:
    print(f"Error processing XACRO file: {e}")
    robot_description_raw = ""

# Set the world file path
world_file_name = 'empty.sdf'
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
            'gz_args': f'-r -v 4 {gz_world_arg}'.strip()
        }.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',      
            '-name', robot_name_in_gazebo,       
            '-allow_renaming', 'true',
            '-x', PythonExpression(f"str({initial_pose[0]})"),
            '-y', PythonExpression(f"str({initial_pose[1]})"),
            '-z', PythonExpression(f"str({initial_pose[2]})"),
            '-R', PythonExpression(f"str({initial_pose[3]})"),
            '-P', PythonExpression(f"str({initial_pose[4]})"),
            '-Y', PythonExpression(f"str({initial_pose[5]})")
        ],
        output='screen'
    )

    gazebo_imu_topic = f'/world/default/model/{robot_name_in_gazebo}/link/imu_link/sensor/imu_sensor/imu'
    ros2_imu_topic = '/trunk_imu'

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{gazebo_imu_topic}@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        remappings=[
            (gazebo_imu_topic, ros2_imu_topic),
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        launch_gazebo,
        robot_state_publisher_node,
        spawn_robot,
        bridge,

    ])