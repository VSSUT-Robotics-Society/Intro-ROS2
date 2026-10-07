import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray


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

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_value = True


class PIDController(Node):
    """Node class wrapping a minimal PID controller."""

    def __init__(self) -> None:
        super().__init__('pid_controller')

        # Encoder joint name
        self.input_joint: str = 'pendulum_joint'

        # Controllable joint name
        self.output_joint: str = 'shaft_joint'

        # PID gains
        self.controllers = [
            PIDControl(
                K=[14.0, 0.0, 0.5]   # Example gains for bob
            ),
            PIDControl(
                K=[0.05, 0.001, 0.0045]   # Example gains for shaft
            )
        ]

        # Target setpoint(s)
        self.setPoint_bob: float = 0.0  # Desired position for bob (radians)
        # Desired position for shaft (radians)
        self.setPoint_shaft: float = 0.0
        self.prev_stamp: float = 0.0

        # Input limit within which system stays on (+- <angle> degrees)
        self.input_limit: float = 60.0  # degrees
        self.input_debounce_limit: float = 5.0     # degrees
        self._is_active: bool = False     # Flag stating whether system is active
        self._counter = 0    # Number of times a controller has ran

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

        # Publisher for telemetry
        self.pid_pub = self.create_publisher(
            Float64MultiArray,
            '/pid_telemetry',
            10
        )
        self.pid = Float64MultiArray()  # Message to publish controller telemetry

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
        # Extract raw positions
        joint_dict: dict[str, float] = {
            name: pos if pos is not None else 0.0
            for name, pos in zip(msg.name, msg.position)
        }

        # Extract joints, normalizing only 'pendulum_joint' to [-pi, pi]
        raw_bob = joint_dict.get('pendulum_joint', 0.0)
        current_bob = (raw_bob + math.pi) % math.tau - math.pi

        # 'shaft_joint' remains unnormalized (e.g., preserving continuous rotation)
        current_shaft = joint_dict.get('shaft_joint', 0.0)

        # Normalize timestamps
        current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not isinstance(current_stamp, float) or current_stamp == 0.0:
            current_stamp = self.prev_stamp    # type checking failure
            return
        dt_msg = current_stamp - self.prev_stamp
        self.prev_stamp = current_stamp  # update prev stamp or next iteration

        # Limit checks
        abs_bob_deg = math.degrees(abs(current_bob))
        if not self._is_active:
            if abs_bob_deg < self.input_debounce_limit:
                self.get_logger().info(
                    f'Pendulum is within limits ({abs_bob_deg:.1f}°). Recovering...'
                )
                self._is_active = True  # System has recovered within debounce limits
        if abs_bob_deg > self.input_limit:
            self._is_active = False     # System exceeded limits, shutdown
            self.get_logger().warning(
                f'Pendulum fell beyond limit ({abs_bob_deg:.1f}°). Resetting...'
            )
            # !Reset simulation to upright (could also switch controllers here)
            self.reset_cmd.data = True
            self.reset_pub.publish(self.reset_cmd)
            return

        # Shaft PID (Runs every 10th iteration)
        error_shaft = self.setPoint_shaft - current_shaft
        if self._counter >= 9:
            self._counter = 0
            self.setPoint_bob = -self.controllers[1].compute_control(
                error_shaft, dt=dt_msg)
            self.setPoint_bob = max(
                min(self.setPoint_bob, math.radians(self.input_debounce_limit)),
                -math.radians(self.input_debounce_limit)
            )

        # Bob PID
        error_bob = self.setPoint_bob - current_bob
        control_bob = self.controllers[0].compute_control(
            error=error_bob, dt=dt_msg)
        self._counter += 1

        # Log the computed control outputs for debugging
        self.pid.data = [
            # Time
            dt_msg,
            # Bob
            self.setPoint_bob,
            current_bob,
            error_bob,
            self.controllers[0].integral,
            # Shaft
            current_shaft,
            error_shaft,
            self.controllers[1].integral
        ]
        self.pid_pub.publish(self.pid)

        # Publish control outputs to appropriate topics
        self.cmd.data = control_bob
        self.publisher.publish(self.cmd)

    def reset_callback(self, msg: Bool) -> None:
        if msg.data:
            # Reset all PID states
            for pid in self.controllers:
                pid.reset()
                self.cmd.data = 0.0  # Stop any force application
                self.publisher.publish(self.cmd)
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
