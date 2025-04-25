import launch
import launch_ros.actions
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    hector_control_node = launch_ros.actions.Node(
        package="hector_control",
        executable="hector_ctrl",
        name="hector_control",
        output="screen",
        parameters=[{"physical_robot": True}]
    )

    shutdown_on_hector_control_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=hector_control_node,  # Monitor this node
            on_exit=[Shutdown()]  # Shutdown entire launch file on exit
        )
    )

    return launch.LaunchDescription([
        hector_control_node,

        launch_ros.actions.Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
        ),

        # launch_ros.actions.Node(
        #     package="state_estimator",
        #     executable="state_estimator_node",
        #     name="state_estimator_node",
        #     parameters=[{"physical_robot": True}]
        # ),

        launch_ros.actions.Node(
            package="laser_comm",
            executable="hector_communication_node",
            name="hector_communication_node",
            # output="screen",
        ),

        shutdown_on_hector_control_exit,  # Ensures all nodes stop when hector_control exits
    ])
