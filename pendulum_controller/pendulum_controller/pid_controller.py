import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool

import math


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Global class for PID control
        class PIDControl:
            def __init__(self, K):
                self.K = K  # PID gains [Kp, Ki, Kd]
                self.integral = 0.0
                self.prev_error = 0.0

            def compute_control(self, error, dt=1.0) -> float:
                # Update integral and derivative terms
                self.integral += error * dt if dt > 0.0 else 0.0
                derivative = (error - self.prev_error) / \
                    dt if dt > 0.0 else 0.0

                # Update previous error for next iteration
                self.prev_error = error

                # Return the PID control output
                return (self.K[0] * error) + (self.K[1] * self.integral) + (self.K[2] * derivative)

        # PID gains for the bob (pendulum)
        self.bob = PIDControl(K=[50.0, 0.0, 0.0])  # Example gains for bob
        self.rail = PIDControl(K=[0.0, 1e-5, 0.0])  # Example gains for rail

        # Target setpoints for bob and rail
        self.setPoint_bob = 0.0  # Desired position for bob (radians)
        self.setPoint_cart = 0.0  # Desired position for rail (meters)

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.topic_callback,
            10
        )

        # Publisher for joint control commands
        self.publisher = self.create_publisher(
            Float64,
            '/joint_control',
            10
        )
        self.cmd = Float64()    # Message to publish control commands

        # Subscriber for reset commands
        self.reset_subscription = self.create_subscription(
            Bool,
            '/world/reset',
            self.reset_callback,
            10
        )

    def topic_callback(self, msg: JointState):
        # Extract current positions
        current_bob = msg.position[msg.name.index(
            'base_joint')] if 'base_joint' in msg.name else 0.0
        current_cart = msg.position[msg.name.index(
            'rail_joint')] if 'rail_joint' in msg.name else 0.0

        # Calculate errors
        error_bob = self.setPoint_bob - current_bob
        error_cart = self.setPoint_cart - current_cart

        # Calculate control outputs using PID
        self.setPoint_bob = self.rail.compute_control(error_cart)
        control_cart = -self.bob.compute_control(error_bob)

        # Log the computed control outputs for debugging
        self.get_logger().info(
            f'Error Bob: {error_bob:.4f}, Control Cart: {control_cart:.4f}')

        # Publish control outputs to appropriate topics
        self.cmd.data = control_cart
        self.publisher.publish(self.cmd)

    def reset_callback(self, msg: Bool):
        if msg.data:
            # Reset all PID states
            for pid in [self.bob, self.rail]:
                pid.integral = 0.0
                pid.prev_error = 0.0
            self.get_logger().info('Reset command received. Cleared PID states')


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()

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
