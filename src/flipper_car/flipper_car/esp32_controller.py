#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_controller')
        
        # Subscriptions
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/flipper/command', self.command_callback, 10)
        
        # Publisher to Gazebo
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.is_upside_down = False
        self.current_command = "STOP"

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("TC1508 4-Quadrant Controller initialized.")

    def command_callback(self, msg: String):
        self.current_command = msg.data

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Inversion threshold (> 90 degrees roll)
        self.is_upside_down = abs(roll) > (math.pi / 2.0)

    def control_loop(self):
        twist = Twist()

        base_speed = 0.5        # m/s
        spin_omega = 2.5        # rad/s (in-place zero-radius spin)
        turn_omega = 1.25       # rad/s (smooth arc)

        flip_drive = -1.0 if self.is_upside_down else 1.0
        flip_steer = -1.0 if self.is_upside_down else 1.0

        if self.current_command == "FORWARD":
            twist.linear.x = base_speed * flip_drive
            twist.angular.z = 0.0

        elif self.current_command == "REVERSE":
            twist.linear.x = -base_speed * flip_drive
            twist.angular.z = 0.0

        elif self.current_command == "SPIN_LEFT":
            twist.linear.x = 0.0
            twist.angular.z = spin_omega * flip_steer

        elif self.current_command == "SPIN_RIGHT":
            twist.linear.x = 0.0
            twist.angular.z = -spin_omega * flip_steer

        elif self.current_command == "FWD_LEFT":
            twist.linear.x = (base_speed * 0.75) * flip_drive
            twist.angular.z = turn_omega * flip_steer

        elif self.current_command == "FWD_RIGHT":
            twist.linear.x = (base_speed * 0.75) * flip_drive
            twist.angular.z = -turn_omega * flip_steer

        elif self.current_command == "REV_LEFT":
            twist.linear.x = -(base_speed * 0.75) * flip_drive
            twist.angular.z = -turn_omega * flip_steer

        elif self.current_command == "REV_RIGHT":
            twist.linear.x = -(base_speed * 0.75) * flip_drive
            twist.angular.z = turn_omega * flip_steer

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