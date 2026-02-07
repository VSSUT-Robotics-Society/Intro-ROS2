import rclpy
import math
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID parameters
        self.kp = 10.0
        self.ki = 1.0
        self.kd = 0.01

        # Safety/robustness parameters
        self.joint_name = 'pendulum_joint'
        self.max_step_rad = 0.2
        self.max_integral = 1.0

        # State variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/stepper/commands',
            10
        )

        self.get_logger().info('PID Controller Node has been started.')
        self.get_logger().info(
            f'PID parameters: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')

    def joint_state_callback(self, msg):
        # Use joint name lookup instead of relying on index ordering
        try:
            joint_index = msg.name.index(self.joint_name)
        except ValueError:
            self.get_logger().warn(
                f'Joint {self.joint_name} not found in JointState.')
            return

        current_angle = msg.position[joint_index]
        desired_angle = 0.0  # Target angle (upright position)

        # Calculate error
        error = math.atan2(math.sin(desired_angle - current_angle),
                           math.cos(desired_angle - current_angle))
        self.get_logger().info(
            f'Error angle of {self.joint_name}: {error:.4f} radians')

        # Get current time
        current_time = self.get_clock().now()

        # Calculate time difference
        if self.last_time is None:
            self.last_time = current_time
            self.previous_error = error
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Update integral and derivative
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0

        # Compute control signal
        control_signal = self.kp * error + self.ki * \
            self.integral + self.kd * derivative

        # Clamp the step size to avoid sudden large position jumps
        if control_signal > self.max_step_rad:
            control_signal = self.max_step_rad
        elif control_signal < -self.max_step_rad:
            control_signal = -self.max_step_rad

        command_position = current_angle + control_signal

        # Publish control signal
        command_msg = Float64MultiArray()
        # Position setpoint for the stepper motor
        command_msg.data = [command_position]
        self.publisher.publish(command_msg)

        # Update previous error and time
        self.previous_error = error
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()
