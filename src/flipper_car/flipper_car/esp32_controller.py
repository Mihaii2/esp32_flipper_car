import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
import math
import time

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_controller')
        
        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/flipper/command', self.command_callback, 10)
        self.pol_sub = self.create_subscription(Bool, '/flipper/polarity', self.polarity_callback, 10)
        
        # Publisher to Gazebo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.servo_angle = 0.0              # 0.0 = Forward, 180.0 = Reverse
        self.target_servo_angle = 0.0
        self.is_upside_down = False
        self.current_command = "STOP"
        
        # Gear Shift Dead-Time (200ms for SG90 physical transit)
        self.shift_in_progress = False
        self.shift_end_time = 0.0
        self.GEAR_SHIFT_DELAY_SEC = 0.20

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("ESP32 Controller logic initialized.")

    def command_callback(self, msg: String):
        self.current_command = msg.data

    def polarity_callback(self, msg: Bool):
        new_angle = 180.0 if msg.data else 0.0
        if new_angle != self.servo_angle:
            self.target_servo_angle = new_angle
            self.shift_in_progress = True
            self.shift_end_time = time.time() + self.GEAR_SHIFT_DELAY_SEC
            self.get_logger().info(
                f"Gear shift initiated! Cutting motor power for {int(self.GEAR_SHIFT_DELAY_SEC*1000)}ms..."
            )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        self.is_upside_down = abs(roll) > (math.pi / 2.0)

    def control_loop(self):
        twist = Twist()

        # 1. Handle Gear Shift Transit / Dead-Time Protection
        if self.shift_in_progress:
            if time.time() >= self.shift_end_time:
                self.servo_angle = self.target_servo_angle
                self.shift_in_progress = False
                self.get_logger().info(f"Gear shift complete. Polarity now at {self.servo_angle}°.")
            else:
                # Force zero output to prevent contact arcing during mechanical swing
                self.cmd_vel_pub.publish(twist)
                return

        # 2. Base Speeds (Differential wheel spacing geometry: R_wheel * omega = v / 2)
        base_speed = 0.5
        wheel_track = 0.2  # approximate chassis track width (m)
        pivot_omega = base_speed / (wheel_track / 2.0)  # Turning rate when 1 wheel is locked

        polarity_dir = -1.0 if self.servo_angle >= 90.0 else 1.0
        flip_drive_mult = -1.0 if self.is_upside_down else 1.0
        flip_steer_mult = -1.0 if self.is_upside_down else 1.0

        effective_dir = polarity_dir * flip_drive_mult

        # 3. Discrete Low-Side MOSFET Kinematics
        if self.current_command == "FORWARD":
            # Both MOSFETs ON (100% PWM)
            twist.linear.x = base_speed * effective_dir
            twist.angular.z = 0.0

        elif self.current_command == "FWD_LEFT":
            # Left side 50% PWM, Right side 100% PWM (gentle arc)
            twist.linear.x = (base_speed * 0.75) * effective_dir
            twist.angular.z = (pivot_omega * 0.5) * flip_steer_mult

        elif self.current_command == "FWD_RIGHT":
            # Right side 50% PWM, Left side 100% PWM (gentle arc)
            twist.linear.x = (base_speed * 0.75) * effective_dir
            twist.angular.z = -(pivot_omega * 0.5) * flip_steer_mult

        elif self.current_command == "FULL_LEFT":
            # Left MOSFET OFF (0% PWM), Right MOSFET ON (100% PWM) -> Pivot around stopped left wheel
            twist.linear.x = (base_speed * 0.5) * effective_dir
            twist.angular.z = pivot_omega * flip_steer_mult

        elif self.current_command == "FULL_RIGHT":
            # Right MOSFET OFF (0% PWM), Left MOSFET ON (100% PWM) -> Pivot around stopped right wheel
            twist.linear.x = (base_speed * 0.5) * effective_dir
            twist.angular.z = -pivot_omega * flip_steer_mult

        else:  # STOP
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()