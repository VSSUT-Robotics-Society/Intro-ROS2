import rclpy
import math
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID parameters
        self.kp: float = 24.0
        self.ki: float = 10.0
        self.kd: float = 0.07
        self.dt: float = 0.02  # Time step for control loop (50 Hz)

        # Safety/robustness parameters
        self.joint_name = 'pendulum_joint'
        self.max_integral = 1.0     # Prevent integral windup
        self.max_command = 2.0      # Max command to stepper motor

        # Switch to recovery if error exceeds 30 degrees
        self.recovery_threshold = math.pi / 6

        # State variables
        self.desired_angle: float = 0.0  # Target angle (upright position)
        self.previous_error = 0.0
        self.integral = 0.0

        # IO variables
        self.current_angle = 0.0
        self.command = Float64MultiArray()

        # Subscribers
        self.input = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.output = self.create_publisher(
            Float64MultiArray,
            '/stepper/commands',
            10
        )

        self.timer = self.create_timer(
            self.dt, self.control_loop)  # 50 Hz control loop

        self.get_logger().info('PID Controller Node has been started.')

        # Start a GUI for tuning
        threading.Thread(target=self.tuning_gui, daemon=True).start()

    def joint_state_callback(self, msg):
        # Use joint name lookup instead of relying on index ordering
        try:
            joint_index = msg.name.index(self.joint_name)
        except ValueError:
            self.get_logger().warn(
                f'Joint {self.joint_name} not found in JointState.')
            return

        # Store current angle for control loop
        self.current_angle = msg.position[joint_index]

    def control_loop(self):
        # Two parts: One is PID control, other is recovery from large errors
        current_time = self.get_clock().now()   # Get current time
        error = self.desired_angle - self.current_angle
        if abs(error) > self.recovery_threshold:
            # If error is large, use energy injection strategy to swing up
            self.get_logger().warn(
                f'Large error detected: {math.degrees(error):.2f} degrees.')
            # TODO: Implement energy injection strategy here
            # Reset integral and derivative terms
            self.integral = 0.0
            self.previous_error = 0.0
            return  # Skip PID control when in recovery mode

        # PID control
        P = self.kp * error
        self.integral += error * self.dt
        # Anti-windup: Clamp integral term, warn if we hit limits
        if abs(self.integral) > self.max_integral:
            self.get_logger().warn(
                f'Integral term {self.integral:.3f} exceeds max limit, clamping.')
        self.integral = max(
            min(self.integral, self.max_integral), -self.max_integral)
        I = self.ki * self.integral
        D = self.kd * (error - self.previous_error) / self.dt
        self.previous_error = error
        command_value = P + I + D
        # Clamp command to max limits, warning if we hit limits
        if abs(command_value) > self.max_command:
            self.get_logger().warn(
                f'Command {command_value:.3f} exceeds max limit, clamping.')
        command_value = max(
            min(command_value, self.max_command), -self.max_command)
        self.command.data = [command_value]
        self.output.publish(self.command)
        self.get_logger().info(
            f'Command: {command_value:.3f}, Error: {math.degrees(error):.2f} degrees')

    def tuning_gui(self):
        # Uses Tkinter for a simple GUI to tune PID parameters (modifies self.kp, self.ki, self.kd)
        import tkinter as tk
        from tkinter import ttk

        # Slider configuration (edit these to change limits and step sizes)
        kp_limits = (0.0, 100.0)
        ki_limits = (0.0, 20.0)
        kd_limits = (0.0, 10.0)
        kp_step = 0.5
        ki_step = 0.05
        kd_step = 0.025

        root = tk.Tk()
        root.title("PID Tuning")
        root.resizable(False, False)
        root.configure(padx=14, pady=12)

        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Header.TLabel", font=("Helvetica", 12, "bold"))
        style.configure("TLabel", font=("Helvetica", 10))
        style.configure("Value.TLabel", font=("Helvetica", 10, "bold"))

        header = ttk.Label(root, text="PID Tuning", style="Header.TLabel")
        header.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

        ttk.Separator(root, orient="horizontal").grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(0, 8)
        )

        frame = ttk.Frame(root, padding=(4, 2))
        frame.grid(row=2, column=0, columnspan=3, sticky="ew")
        frame.columnconfigure(1, weight=1)

        kp_value = tk.StringVar(value=f"{self.kp:.2f}")
        ki_value = tk.StringVar(value=f"{self.ki:.2f}")
        kd_value = tk.StringVar(value=f"{self.kd:.2f}")

        def update_kp(val):
            self.kp = float(val)
            kp_value.set(f"{self.kp:.2f}")

        def update_ki(val):
            self.ki = float(val)
            ki_value.set(f"{self.ki:.2f}")

        def update_kd(val):
            self.kd = float(val)
            kd_value.set(f"{self.kd:.2f}")

        ttk.Label(frame, text="Kp").grid(
            row=0, column=0, sticky="w", padx=(0, 8))
        kp_slider = ttk.Scale(
            frame, from_=kp_limits[0], to=kp_limits[1], orient="horizontal", command=update_kp
        )
        kp_slider.set(self.kp)
        kp_slider.grid(row=0, column=1, sticky="ew")
        kp_value_label = ttk.Label(
            frame, textvariable=kp_value, style="Value.TLabel")
        kp_value_label.grid(
            row=0, column=2, sticky="e", padx=(10, 0)
        )

        ki_label = ttk.Label(frame, text="Ki")
        ki_label.grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(6, 0))
        ki_slider = ttk.Scale(
            frame, from_=ki_limits[0], to=ki_limits[1], orient="horizontal", command=update_ki
        )
        ki_slider.set(self.ki)
        ki_slider.grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ki_value_label = ttk.Label(
            frame, textvariable=ki_value, style="Value.TLabel")
        ki_value_label.grid(
            row=1, column=2, sticky="e", padx=(10, 0), pady=(6, 0)
        )

        kd_label = ttk.Label(frame, text="Kd")
        kd_label.grid(row=2, column=0, sticky="w", padx=(0, 8), pady=(6, 0))
        kd_slider = ttk.Scale(
            frame, from_=kd_limits[0], to=kd_limits[1], orient="horizontal", command=update_kd
        )
        kd_slider.set(self.kd)
        kd_slider.grid(row=2, column=1, sticky="ew", pady=(6, 0))
        kd_value_label = ttk.Label(
            frame, textvariable=kd_value, style="Value.TLabel")
        kd_value_label.grid(
            row=2, column=2, sticky="e", padx=(10, 0), pady=(6, 0)
        )

        style.configure("Focused.TLabel", font=(
            "Helvetica", 10, "bold"), foreground="#1d4ed8")
        style.configure("Focused.Value.TLabel", font=(
            "Helvetica", 10, "bold"), foreground="#1d4ed8")
        style.configure("Focused.Horizontal.TScale", troughcolor="#93c5fd")

        sliders = [kp_slider, ki_slider, kd_slider]
        step_sizes = [kp_step, ki_step, kd_step]
        limits = [kp_limits, ki_limits, kd_limits]
        name_labels = [ttk.Label(frame, text="Kp"), ki_label, kd_label]
        name_labels[0].grid(row=0, column=0, sticky="w", padx=(0, 8))
        value_labels = [kp_value_label, ki_value_label, kd_value_label]

        def focus_slider(index):
            if 0 <= index < len(sliders):
                sliders[index].focus_set()

        def current_slider_index():
            current = root.focus_get()
            for idx, slider in enumerate(sliders):
                if current is slider:
                    return idx
            return None

        def on_arrow(event):
            idx = current_slider_index()
            if idx is None:
                return
            if event.keysym in ("Up", "Down"):
                delta = -1 if event.keysym == "Up" else 1
                focus_slider((idx + delta) % len(sliders))
                return "break"
            if event.keysym in ("Left", "Right"):
                step = step_sizes[idx]
                delta = -step if event.keysym == "Left" else step
                slider = sliders[idx]
                value = slider.get() + delta
                min_val, max_val = limits[idx]
                value = max(min_val, min(max_val, value))
                slider.set(value)
                return "break"
            return None

        def set_focus_style(index, focused):
            label_style = "Focused.TLabel" if focused else "TLabel"
            value_style = "Focused.Value.TLabel" if focused else "Value.TLabel"
            name_labels[index].configure(style=label_style)
            value_labels[index].configure(style=value_style)
            sliders[index].configure(
                style="Focused.Horizontal.TScale" if focused else "Horizontal.TScale")

        for slider in sliders:
            slider.configure(takefocus=True)
            slider.bind("<Up>", on_arrow)
            slider.bind("<Down>", on_arrow)
            slider.bind("<Left>", on_arrow)
            slider.bind("<Right>", on_arrow)

        for idx, slider in enumerate(sliders):
            slider.bind("<FocusIn>", lambda _e,
                        i=idx: set_focus_style(i, True))
            slider.bind("<FocusOut>", lambda _e,
                        i=idx: set_focus_style(i, False))

        focus_slider(0)

        root.mainloop()     # This is a blocking call, but runs in a separate thread


def main(args=None):
    rclpy.init(args=args)
    pid_controller_node = PIDControllerNode()
    rclpy.spin(pid_controller_node)
    pid_controller_node.destroy_node()
    rclpy.shutdown()
