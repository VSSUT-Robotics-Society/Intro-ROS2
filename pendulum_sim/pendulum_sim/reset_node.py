import subprocess
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from ros_gz_interfaces.msg import WorldControl, WorldReset
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import math


class ResetWorldNode(Node):
    def __init__(self):
        super().__init__('reset_world_node')

        # Target joint and threshold settings
        self.TARGET_JOINT = 'base_joint'
        self.THRESHOLD = math.radians(75.0)     # Absolute threshold value

        # Service client for Gazebo world control
        self.reset_client = self.create_client(
            ControlWorld,
            '/world/empty/control'
        )

        # RViz2 Reset Time Client
        self.rviz_reset_client = self.create_client(
            Empty,
            '/rviz2/reset_time'
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.topic_callback,
            10
        )

        # Subscriber for reset commands
        self.reset_subscription = self.create_subscription(
            Bool,
            '/world/reset',
            self.reset_callback,
            10
        )

        # Publisher for reset commands
        self.reset_publisher = self.create_publisher(
            Bool,
            '/world/reset',
            10
        )
        self.reset_msg = Bool()
        # Initial state: not resetting
        self.reset_msg.data = False
        self.reset_publisher.publish(self.reset_msg)

        self.get_logger().info(
            f'Monitoring topic for threshold > {self.THRESHOLD}...')

    def topic_callback(self, msg: JointState):
        # Ignore incoming data if a reset operation is currently running
        if self.reset_msg.data:
            return

        current_val = msg.position[msg.name.index(
            self.TARGET_JOINT)] if self.TARGET_JOINT in msg.name else 0.0

        # Check threshold condition
        if abs(current_val) > self.THRESHOLD:
            self.get_logger().warn(
                f'Threshold exceeded! Value: {current_val:.2f} (Limit: {self.THRESHOLD}). Triggering reset...'
            )
            self.execute_reset_sequence()

    def reset_callback(self, msg: Bool):
        if msg.data:
            if not self.reset_msg.data:  # Only trigger if not already resetting
                self.get_logger().info('External reset command received. Triggering reset...')
                self.execute_reset_sequence()

    def execute_reset_sequence(self):
        self.reset_msg.data = True
        self.reset_publisher.publish(self.reset_msg)

        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /world/empty/control not available!')

            self.reset_msg.data = False
            self.reset_publisher.publish(self.reset_msg)

            return

        # Prepare reset request
        req = ControlWorld.Request()
        req.world_control = WorldControl(
            reset=WorldReset(all=True)
        )

        # Call service asynchronously with a callback (non-blocking)
        future = self.reset_client.call_async(req)
        future.add_done_callback(self.reset_done_callback)

    def reset_rviz_time(self):
        if self.rviz_reset_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            rviz_future = self.rviz_reset_client.call_async(req)
            self.get_logger().info('Triggered /rviz2/reset_time service.')
        else:
            self.get_logger().warn('/rviz2/reset_time service not available, TF warnings may persist.')

    def reset_done_callback(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                self.reset_rviz_time()  # Reset RViz time after world reset
                self.get_logger().info('World reset successful. Re-spawning entity...')
                self.spawn_entity('Pendulum')
            else:
                self.get_logger().error('World reset failed.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            self.reset_msg.data = False
            self.reset_publisher.publish(self.reset_msg)

    def spawn_entity(self, entity_name: str, entity_pose: Pose = Pose()):
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', '/robot_description',
            '-name', entity_name,
            '-pose', (
                f'{entity_pose.position.x} {entity_pose.position.y} {entity_pose.position.z} '
                f'{entity_pose.orientation.x} {entity_pose.orientation.y} '
                f'{entity_pose.orientation.z} {entity_pose.orientation.w}'
            )
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            self.get_logger().info(f"Successfully spawned '{entity_name}'.")
        else:
            self.get_logger().error(f"Spawn failed: {result.stderr}")


def main(args=None):
    rclpy.init(args=args)
    node = ResetWorldNode()

    try:
        # Spin keeps the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user. Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
