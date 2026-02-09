import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TestMotorNode(Node):
    def __init__(self):
        super().__init__('test_motor_node')

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/stepper/commands',
            10
        )

        self.get_logger().info('Starting revolution test...')

        # Publish commands to rotate the motor in one direction
        for i in range(10):
            msg = Float64MultiArray()
            msg.data = [1.0]  # Command to rotate in one direction
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            time.sleep(0.5)

        # Publish commands to rotate the motor in the opposite direction
        for i in range(10):
            msg = Float64MultiArray()
            msg.data = [-1.0]  # Command to rotate in the opposite direction
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            time.sleep(0.5)

        # Stop the motor
        msg = Float64MultiArray()
        msg.data = [0.0]  # Command to stop the motor
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    test_motor_node = TestMotorNode()
    rclpy.spin(test_motor_node)
    test_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
