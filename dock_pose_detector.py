#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException
import tf2_geometry_msgs
import time

class DetectedDockPosePublisher(Node):
    def __init__(self):
        super().__init__('detected_dock_pose_publisher')

        # Declare parameters with default values
        self.declare_parameter('parent_frame', 'camera')
        self.declare_parameter('child_frame', 'charge')
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get the values of our parameters
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Create a transform buffer to store and look up transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a publisher for the dock pose
        self.dock_pose_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)

        # Create a timer to trigger the publishing of the dock pose at the specified rate
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Log that we've successfully initialized
        self.get_logger().info(f"Detected dock pose publisher initialized with parent frame: '{self.parent_frame}' and child frame: '{self.child_frame}'")

    def timer_callback(self):
        """Timer callback that publishes the latest dock pose"""
        dock_pose = PoseStamped()
        dock_pose.header.stamp = self.get_clock().now().to_msg()
        dock_pose.header.frame_id = self.parent_frame

        try:
            # Look up the transform
            transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())

            # Copy the translation from the transform to the pose
            dock_pose.pose.position.x = transform.transform.translation.x
            dock_pose.pose.position.y = transform.transform.translation.y
            dock_pose.pose.position.z = transform.transform.translation.z

            # Copy the rotation from the transform to the pose
            dock_pose.pose.orientation = transform.transform.rotation

            # Publish the dock pose
            self.dock_pose_pub.publish(dock_pose)
            self.get_logger().info(f"Published dock pose: {dock_pose.pose}")

        except TransformException as e:
            # If we can't get the transform, log it at debug level to avoid spamming
            self.get_logger().debug(f"Could not get transform: {e}")
            return

def main(args=None):
    # Initialize ROS2 and the node
    rclpy.init(args=args)
    node = DetectedDockPosePublisher()

    # Spin the node so that it can keep publishing and handling callbacks
    rclpy.spin(node)

    # Clean up and shutdown ROS2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

