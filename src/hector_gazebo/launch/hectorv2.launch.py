import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro

# --- 配置 ---

# Robot name (用于 Gazebo 中的实体名称)
# !!! 重要: 这个名称必须与构建 ros_gz_bridge 参数时使用的模型名称一致 !!!
robot_name_in_gazebo = 'hector_description' # 或者你希望在 Gazebo 中使用的实际名称

# 机器人初始位姿 [x, y, z, roll, pitch, yaw]
initial_pose = [0.0, 0.0, 100.0, 0.0, 1.57, 0.0]

# 获取 hector_gazebo 包的路径
hector_gazebo_pkg_path = get_package_share_directory('hector_gazebo')

# 配置机器人的 URDF/XACRO 文件
robot_description_subpath = 'assets/models/xacro/robot.xacro'
xacro_file = os.path.join(hector_gazebo_pkg_path, robot_description_subpath)
try:
    # 确保 xacro 处理能够找到所有包含的文件
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    print("XACRO processing successful.")
except Exception as e:
    print(f"Error processing XACRO file: {e}")
    # 在出错时提供一个默认的空描述，防止后续节点失败，但仿真会缺少机器人
    robot_description_raw = ""
    # 或者直接抛出异常退出
    # raise e

# 设置世界文件的路径 (使用修正后的 .sdf 文件)
world_file_name = 'empty.sdf' # 确认你已经创建并保存了修正后的 SDF 文件
world_path = os.path.join(hector_gazebo_pkg_path, 'assets', 'models', 'worlds', world_file_name)
# 检查世界文件是否存在
if not os.path.exists(world_path):
    print(f"警告：世界文件未找到: {world_path}. Gazebo Sim 将尝试加载默认世界。")
    gz_world_arg = ""
else:
    gz_world_arg = world_path

# --- Launch Description 生成 ---

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # === 启动 Gazebo Sim (Ignition) ===
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            # '-r' 启动时暂停, '-v 4' 详细日志, 最后是世界文件路径
            'gz_args': f'-r -v 4 {gz_world_arg}'.strip()
        }.items()
    )

    # === 启动 Robot State Publisher ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # === 在 Gazebo Sim 中生成机器人 ===
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',      # 从 ROS topic 获取模型描述
            '-name', robot_name_in_gazebo,       # 在 Gazebo 中创建的实体名称
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

    # === 添加 ros_gz_bridge ===
    # 桥接 Gazebo Transport 的 IMU 主题到 ROS 2
    # !! 注意: 请根据实际情况核对 Gazebo 中的主题名称 !!
    # Gazebo 主题名称格式通常为: /world/<world_name>/model/<model_name>/link/<link_name>/sensor/<sensor_name>/<topic_type>
    # 在这里: world_name='default', model_name=robot_name_in_gazebo, link_name='imu_link', sensor_name='imu_sensor', topic_type='imu'
    gazebo_imu_topic = f'/world/default/model/{robot_name_in_gazebo}/link/imu_link/sensor/imu_sensor/imu'
    ros2_imu_topic = '/trunk_imu' # 你希望在 ROS 2 中使用的 IMU 主题名称

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 桥接 IMU 数据
            f'{gazebo_imu_topic}@sensor_msgs/msg/Imu@gz.msgs.IMU'
            # 如果有其他传感器 (如接触传感器)，在此处添加更多桥接参数
            # 例如，假设有一个接触传感器在 'L_ankle_link' 上，名为 'left_foot_contact'
            # f'/world/default/model/{robot_name_in_gazebo}/link/L_ankle_link/sensor/left_foot_contact/contact@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts',
        ],
        remappings=[
            # 将默认的 ROS 2 主题名称重映射为我们期望的名称
            (gazebo_imu_topic, ros2_imu_topic),
            # 如果添加了其他桥接，也在此处添加重映射
            # (f'/world/default/model/{robot_name_in_gazebo}/link/L_ankle_link/sensor/left_foot_contact/contact', '/left_foot_contact'),
        ],
        output='screen'
    )

    # === (可选) 加载关节状态发布器和轨迹控制器 ===
    # 如果你使用了 ros2_control，你需要加载相应的控制器
    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )
    #
    # joint_trajectory_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    # )
    # (你需要确保你的配置包中定义了 "joint_trajectory_controller")


    # === 定义 LaunchDescription ===
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # 启动 Gazebo
        launch_gazebo,

        # 启动 Robot State Publisher
        robot_state_publisher_node,

        # 生成机器人模型到 Gazebo
        spawn_robot,

        # 启动 Bridge
        bridge,

        # (可选) 启动 ros2_control 控制器 Spawners
        # joint_state_broadcaster_spawner,
        # joint_trajectory_controller_spawner,

    ])