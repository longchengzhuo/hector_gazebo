#!/usr/bin/env python3
"""
Author: Omar Kolt
Date: June 2, 2024
Contact: omarkolt@hotmail.com
Description: omarkolt@hotmail.com
# Description: This puiblisher node is used to update the position of the robot TF for RViz
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class OdomMapPublisher(Node):
    def __init__(self):
        super().__init__('odom_map_publisher')
        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self) 
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_transform)

    def model_states_callback(self, msg):
        try:
            index = msg.name.index('hector_description')
            pose = msg.pose[index]

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation

            # self.get_logger().info(f"Publishing pose: {pose.position.x}, {pose.position.y}, {pose.position.z}")

            self.tf_broadcaster.sendTransform(transform)  
        except ValueError:
            self.get_logger().warn('Robot name \'hector_description\' not found in model states')


def main(args=None):
    rclpy.init(args=args)
    node = OdomMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
