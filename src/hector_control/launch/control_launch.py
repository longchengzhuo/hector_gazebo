import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="hector_control",
            executable="hector_ctrl",
            name="hector_control",
            output="screen",
            parameters=[{"simulated_robot": True}]
        ),

        launch_ros.actions.Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
        ),

        launch_ros.actions.Node(
            package="state_estimator",
            executable="state_estimator_node",
            name="state_estimator_node",
            # output="screen",
            parameters=[{"simulated_robot": True}]
        ),

    ])
