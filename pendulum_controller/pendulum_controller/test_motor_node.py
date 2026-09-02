import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class TestMotorNode(Node):
    def __init__(self):
        super().__init__('test_motor_node')

        self.publisher = self.create_publisher(
            Float64,
            '/joint_control',
            10
        )

        self.get_logger().info('Starting revolution test...')

    def run_test(self):
        msg = Float64()

        # Rotate forward
        for _ in range(10):
            msg.data = 0.1
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            time.sleep(0.5)

        # Rotate backward
        for _ in range(10):
            msg.data = -0.1
            self.publisher.publish(msg)
            self.get_logger().info(f'Published command: {msg.data}')
            time.sleep(0.5)

        # Stop
        msg.data = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {msg.data}')
        self.get_logger().info('Revolution test completed. Exiting node...')


def main(args=None):
    rclpy.init(args=args)
    test_motor_node = TestMotorNode()

    # Run the test sequence directly
    test_motor_node.run_test()

    # Clean shutdown without spin()
    test_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
