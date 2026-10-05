import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


class PIDControl:
    """Class defining a minimal PID controller."""

    def __init__(self, K: list[float]) -> None:
        self.K: list[float] = K  # PID gains [Kp, Ki, Kd]
        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.integral_limit: float = 1.0  # Limit for integral term to prevent windup
        self.first_value: bool = True   # Flags the first value

    def compute_control(self, error, dt=1.0) -> float:
        if self.first_value:
            self.integral = 0.0     # Keep zero; risk of large dt spike here
            derivative = 0.0    # Keep zero; we don't know what prev value was
            self.first_value = False
        else:
            # Update integral term
            self.integral += error * dt if dt > 0.0 else 0.0
            # Limit the integral term to prevent windup
            self.integral = max(-self.integral_limit,
                                min(self.integral, self.integral_limit))
            # Update derivative term
            derivative = (error - self.prev_error) / \
                dt if dt > 0.0 else 0.0

        # Update previous error for next iteration
        self.prev_error = error

        # Return the PID control output
        return (self.K[0] * error) + (self.K[1] * self.integral) + (self.K[2] * derivative)


class PIDController(Node):
    """Node class wrapping a minimal PID controller."""

    def __init__(self) -> None:
        super().__init__('pid_controller')

        # Encoder joint name
        self.joint: str = 'pendulum_joint'

        # PID gains for the bob (pendulum)
        self.bob: PIDControl = PIDControl(
            K=[1.75, 0.04, 0.1])  # Example gains for bob

        # Target setpoint(s)
        self.setPoint_bob: float = 0.0  # Desired position for bob (radians)
        self.prev_stamp: float = 0.0

        # Input limit within which system stays on (+- <angle> degrees)
        self.input_limit: float = 180.0  # degrees
        self.input_debounce_limit: float = 30.0     # degrees
        self._is_active: bool = True     # Flag stating whether system is active

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
        self.cmd: Float64 = Float64()    # Message to publish control commands

        # Publisher for reset commands
        self.reset_pub = self.create_publisher(
            Bool,
            '/world/reset',
            10
        )
        self.reset_cmd: Bool = Bool()   # Message to publish reset commands

        # Subscriber for reset commands
        self.reset_subscription = self.create_subscription(
            Bool,
            '/world/reset',
            self.reset_callback,
            10
        )

    def topic_callback(self, msg: JointState):
        # Extract current positions
        current_bob = msg.position[msg.name.index(  # type: ignore
            self.joint)] if self.joint in msg.name else 0.0
        # Normalize the angle to be within [-pi, pi]
        current_bob = math.atan2(math.sin(current_bob), math.cos(current_bob))
        # Normalize timestamps
        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not isinstance(current_stamp, float):
            current_stamp = self.prev_stamp    # type checking failure

        # Calculate errors
        error_bob = self.setPoint_bob - current_bob
        dt_bob = current_stamp - self.prev_stamp
        self.prev_stamp = current_stamp  # update prev stamp or next iteration

        # Limit checks
        abs_bob_rad = math.degrees(abs(current_bob))
        if not self._is_active:
            if abs_bob_rad < self.input_debounce_limit:
                self._is_active = True  # System has recovered within debounce limits
        if abs_bob_rad > self.input_limit:
            self.get_logger().warning(
                f'Controller input exceeded limit +-{self.input_limit} degrees. Resetting...'
            )
            self._is_active = False     # System exceeded limits, shutdown
            # !Reset simulation to upright (could also switch controllers here)
            self.reset_cmd.data = True
            self.reset_pub.publish(self.reset_cmd)
            return

        # Calculate control outputs using PID
        control_bob = self.bob.compute_control(error=error_bob, dt=dt_bob)

        # Log the computed control outputs for debugging
        self.get_logger().info(
            f'Error Bob: {error_bob:.4f}, Control Bob: {control_bob:.4f}')

        # Publish control outputs to appropriate topics
        self.cmd.data = control_bob
        self.publisher.publish(self.cmd)

    def reset_callback(self, msg: Bool) -> None:
        if msg.data:
            # Reset all PID states
            self.bob.integral = 0.0
            self.bob.prev_error = 0.0
            self.bob.first_value = True
            self.get_logger().info('Reset command received. Cleared PID states')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PIDController()

    try:
        # Spin keeps the node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Replaced logger with standard print to avoid frame-inspection crash during SIGINT
        print('\n[INFO] Node interrupted by user. Shutting down...')
    finally:
        node.destroy_node()
        # Ensure rclpy hasn't already been shut down by another signal
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
