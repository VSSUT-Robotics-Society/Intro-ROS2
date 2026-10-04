import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial


class EncoderNode(Node):
    """Node class defining Input topic publishing arduino data."""

    def __init__(self):
        super().__init__('encoder_node')

        self.timer = self.create_timer(1.0/10, self.publish_topics)  # 10 Hz

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Initialize Variables
        self.enc_ticks: list[int] = [0]*2  # [angX, angY]

        # Initialize Joint State
        self.joint_msg: JointState = JointState()
        self.joint_msg.name = ['base_joint']
        self.joint_msg.position = [0.0]

        # Initialize Serial Port
        self.serial_port: str = '/dev/ttyACM0'
        try:
            self.serial = serial.Serial(self.serial_port, 115200, timeout=1)
            self.serial.flush()  # Clear buffer

            # Wait for Arduino to initialize
            timer = time.monotonic()
            while self.serial.read() != b'!':
                if time.monotonic() - timer > 8:
                    self.get_logger().error(
                        f'Failed to initialize serial port {self.serial_port}')
                    self.destroy_node()
                    return
                self.serial.write(b'?')
            self.serial.write(b'!')  # Acknowledge
            self.serial.flush()
            self.get_logger().info(
                f'Serial port {self.serial_port} opened successfully')

            self.serial_thread = threading.Thread(target=self.serial_read)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        except serial.SerialException as e:
            self.get_logger().warning(
                f'Failed to open serial port {self.serial_port}: {e}')
            self.destroy_node()

    def serial_read(self):
        while rclpy.ok():
            try:
                data = self.serial.readline().decode().strip()
                if data.startswith('{') and data.endswith('}'):
                    data = data[1:-1].split('|')
                    if len(data) == 2:  # 4 if yaw is not published
                        self.enc_ticks = [int(data[0])]
                    else:
                        self.get_logger().warning(f'Invalid format: {data}')
                elif data == '!':
                    self.get_logger().error('Acknowledge received during Operation!')
                else:
                    self.get_logger().warning(f'Invalid data: {data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break

    def publish_topics(self):
        # Publish Joint States
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        # Convert ticks to radians
        self.joint_msg.position = [
            (self.enc_ticks[0] / 1000) * 2.0 * math.pi
        ]
        self.joint_pub.publish(self.joint_msg)

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


# Main Function
def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[INFO] Node interrupted by user. Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
