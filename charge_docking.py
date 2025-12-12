#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from opennav_docking_msgs.action import DockRobot
from nav2_msgs.action import NavigateToPose
import tf2_geometry_msgs
from tf2_ros import TransformException, Buffer, TransformListener
import math

class DockingClient(LifecycleNode):  # Inherit from LifecycleNode instead of Node
    def __init__(self):
        super().__init__('docking_client')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')  # Action client for navigation
        self.dock_client = ActionClient(self, DockRobot, '/dock_robot')  # Action client for docking

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        q = Quaternion()
        q.x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        q.y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        q.z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        q.w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return q

    def send_navigation_goal(self, x, y, yaw):
        """Send a goal to move the robot to the docking station."""
        self.get_logger().info(f"Navigating to docking station at ({x}, {y}, {yaw})...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Convert yaw to quaternion
        q = self.euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation = q  # Set quaternion orientation

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result().accepted:
            self.get_logger().info("Navigation goal accepted, waiting for result...")
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.result().status == 4:  # Status 4 = SUCCESS
                self.get_logger().info("Arrived at docking station!")
                return True
            else:
                self.get_logger().error("Navigation failed!")
                return False
        else:
            self.get_logger().error("Navigation goal rejected!")
            return False

    def send_dock_goal(self, dock_pose):
        """Send a docking request to the docking server."""
        self.get_logger().info("Requesting docking...")

        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = False  # You are using dock_pose, not dock_id
        goal_msg.dock_pose = dock_pose  # Set the dock pose here
        goal_msg.dock_type = 'nova_carter_dock'  # Ensure this matches your dock type
        goal_msg.max_staging_time = 1000.0  # Optional, can adjust based on needs
        goal_msg.navigate_to_staging_pose = True  # Optional, can adjust based on needs

        self.dock_client.wait_for_server()
        future = self.dock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result().accepted:
            self.get_logger().info("Docking request accepted, waiting for result...")
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            if result_future.result().status == 4:  # Status 4 = SUCCESS
                self.get_logger().info("Docking successful!")
                return True
            else:
                self.get_logger().error("Docking failed!")
                return False
        else:
            self.get_logger().error("Docking request rejected!")
            return False

def main():
    rclpy.init()
    node = DockingClient()

    # Step 1: Navigate to the charging dock
    charging_x = 2.0
    charging_y = 0.0
    charging_yaw = 0.0  # Angle in radians
    if node.send_navigation_goal(charging_x, charging_y, charging_yaw):
        # Step 2: Create a dock_pose message for docking
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = "map"
        dock_pose.header.stamp = node.get_clock().now().to_msg()
        dock_pose.pose.position.x = 2.0
        dock_pose.pose.position.y = -0.5
        dock_pose.pose.position.z = 0.0
        dock_pose.pose.orientation.x = 0.0
        dock_pose.pose.orientation.y = 0.0
        dock_pose.pose.orientation.z = 0.0
        dock_pose.pose.orientation.w = 1.0

        # Step 3: Dock the robot
        node.send_dock_goal(dock_pose)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

