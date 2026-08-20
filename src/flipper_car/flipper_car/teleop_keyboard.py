#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pynput import keyboard

BANNER = """
-------------------------------------------------------
🎮 Flipper Car Multi-Key Teleop (Digital Twin Synced)
-------------------------------------------------------
Controls:
   W          : Drive Forward
   S          : Drive Reverse
   A          : Spin Left
   D          : Spin Right
   W + A      : Forward-Left Curve
   W + D      : Forward-Right Curve
   S + A      : Reverse-Left Curve
   S + D      : Reverse-Right Curve
   (No keys)  : Auto Stop / Coast

Gears:
   1          : 25% Power (Slow/Precision)
   2          : 50% Power
   3          : 75% Power
   4          : 100% Power (Ludicrous)

   CTRL-C     : Quit
-------------------------------------------------------
"""

class MultiKeyTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(String, '/flipper/command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pressed_keys = set()
        self.last_cmd = "STOP"
        
        self.gear_multiplier = 0.25
        self.current_gear_label = "1 (25%)"
        self.last_sent_gear = "GEAR_1"
        
        self.base_speed = 0.6
        self.spin_omega = 2.5
        self.turn_omega = 1.2
        
        print(BANNER)
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                if ch in ['1', '2', '3', '4']:
                    gear_map = {
                        '1': (0.25, 'GEAR_1', '1 (25%)'),
                        '2': (0.50, 'GEAR_2', '2 (50%)'),
                        '3': (0.75, 'GEAR_3', '3 (75%)'),
                        '4': (1.00, 'GEAR_4', '4 (100%)')
                    }
                    self.gear_multiplier, self.last_sent_gear, self.current_gear_label = gear_map[ch]
                    
                    # Send gear update to ESP32 immediately
                    msg = String()
                    msg.data = self.last_sent_gear
                    self.cmd_pub.publish(msg)
                    
                    sys.stdout.write(f"\r>>> GEAR SWITCHED: {self.current_gear_label:<10} | Motion: {self.last_cmd:<12}\n")
                    sys.stdout.flush()
                else:
                    self.pressed_keys.add(ch)
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                self.pressed_keys.discard(ch)
        except AttributeError:
            pass

    def resolve_command(self) -> str:
        w = 'w' in self.pressed_keys
        s = 's' in self.pressed_keys
        a = 'a' in self.pressed_keys
        d = 'd' in self.pressed_keys

        if w and not s:
            if a and not d: return "FWD_LEFT"
            if d and not a: return "FWD_RIGHT"
            return "FORWARD"

        if s and not w:
            if a and not d: return "REV_LEFT"
            if d and not a: return "REV_RIGHT"
            return "REVERSE"

        if a and not d: return "SPIN_LEFT"
        if d and not a: return "SPIN_RIGHT"

        return "STOP"

    def update_and_publish(self):
        current_cmd = self.resolve_command()
        
        # 1. Send hardware command to ESP32
        cmd_msg = String()
        cmd_msg.data = current_cmd
        self.cmd_pub.publish(cmd_msg)

        # 2. Compute and publish scaled Twist directly to Gazebo
        twist = Twist()
        scale = self.gear_multiplier

        if current_cmd == "FORWARD":
            twist.linear.x = self.base_speed * scale
        elif current_cmd == "REVERSE":
            twist.linear.x = -self.base_speed * scale
        elif current_cmd == "SPIN_LEFT":
            twist.angular.z = self.spin_omega * scale
        elif current_cmd == "SPIN_RIGHT":
            twist.angular.z = -self.spin_omega * scale
        elif current_cmd == "FWD_LEFT":
            twist.linear.x = (self.base_speed * 0.75) * scale
            twist.angular.z = self.turn_omega * scale
        elif current_cmd == "FWD_RIGHT":
            twist.linear.x = (self.base_speed * 0.75) * scale
            twist.angular.z = -self.turn_omega * scale
        elif current_cmd == "REV_LEFT":
            twist.linear.x = -(self.base_speed * 0.75) * scale
            twist.angular.z = -self.turn_omega * scale
        elif current_cmd == "REV_RIGHT":
            twist.linear.x = -(self.base_speed * 0.75) * scale
            twist.angular.z = self.turn_omega * scale

        self.cmd_vel_pub.publish(twist)

        if current_cmd != self.last_cmd:
            sys.stdout.write(f"\rCurrent Command: {current_cmd:<12} | Active Gear: {self.current_gear_label:<10}")
            sys.stdout.flush()
            self.last_cmd = current_cmd

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MultiKeyTeleop()

    listener = keyboard.Listener(
        on_press=teleop_node.on_press,
        on_release=teleop_node.on_release
    )
    listener.start()

    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Halt both hardware and simulation on exit
        stop_cmd = String()
        stop_cmd.data = 'STOP'
        teleop_node.cmd_pub.publish(stop_cmd)
        teleop_node.cmd_vel_pub.publish(Twist())
        
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()