import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_controller')
        
        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publisher to Gazebo DiffDrive
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ESP32 State variables
        self.servo_angle = 0.0          # 0 deg = Forward polarity, 180 deg = Reverse polarity
        self.is_upside_down = False      # Replaces SW-520D mechanical switch state
        self.current_command = "STOP"   # e.g., "FORWARD", "FWD_LEFT", "FWD_RIGHT", "FULL_LEFT", "FULL_RIGHT"

        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz loop
        self.get_logger().info("ESP32 Logic Node Running. Ready for discrete commands.")

    def imu_callback(self, msg: Imu):
        # Calculate roll from quaternion to detect if the car is flipped
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # If absolute roll is greater than ~90 degrees (pi/2 radians), car is inverted
        self.is_upside_down = abs(roll) > (math.pi / 2.0)

    def set_servo_polarity(self, angle_degrees: float):
        self.servo_angle = angle_degrees
        self.get_logger().info(f"SG90 Servo rotated to {angle_degrees} deg (Polarity switched).")

    def control_loop(self):
        twist = Twist()
        base_speed = 0.4
        turn_speed = 1.2

        # 1. Determine base polarity multiplier from SG90 Servo position
        polarity_dir = -1.0 if self.servo_angle >= 170.0 else 1.0

        # 2. Invert steering direction if flipped so left/right remain driver-oriented
        steer_dir = -1.0 if self.is_upside_down else 1.0

        # 3. Apply your discrete steering states
        if self.current_command == "FORWARD":
            twist.linear.x = base_speed * polarity_dir
            twist.angular.z = 0.0
        elif self.current_command == "FWD_LEFT":
            twist.linear.x = base_speed * polarity_dir
            twist.angular.z = (turn_speed * 0.5) * steer_dir
        elif self.current_command == "FWD_RIGHT":
            twist.linear.x = base_speed * polarity_dir
            twist.angular.z = -(turn_speed * 0.5) * steer_dir
        elif self.current_command == "FULL_LEFT":
            twist.linear.x = 0.0
            twist.angular.z = turn_speed * steer_dir
        elif self.current_command == "FULL_RIGHT":
            twist.linear.x = 0.0
            twist.angular.z = -turn_speed * steer_dir
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Controller()
    
    # Example demo: simulate driving forward-right for demonstration
    node.current_command = "FWD_RIGHT"
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
