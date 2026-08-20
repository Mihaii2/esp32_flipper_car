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
   W + A      : Sharp Forward-Left Curve
   W + D      : Sharp Forward-Right Curve
   S + A      : Sharp Reverse-Left Curve
   S + D      : Sharp Reverse-Right Curve
   (No keys)  : Auto Stop / Coast

Gears & Boost:
   1          : 55% PWM (Precision / Low)
   2          : 75% PWM (Cruise / Mid)
   3          : 100% PWM (Turbo / High)
   SHIFT (Hold): ⚡ Instant NOS / Turbo Boost (100%)

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
        
        # Selected base gear
        self.base_gear_multiplier = 0.55
        self.base_gear_cmd = "GEAR_1"
        self.base_gear_label = "1 (55%)"
        
        # Shift Boost State
        self.is_boosted = False
        self.last_sent_gear_cmd = "GEAR_1"
        
        self.base_speed = 0.6
        self.spin_omega = 2.5
        self.turn_omega = 2.2
        
        print(BANNER)
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def on_press(self, key):
        # Handle Shift (Boost) Press
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if not self.is_boosted:
                self.is_boosted = True
                self.send_gear_update("GEAR_3")
                sys.stdout.write(f"\r>>> ⚡ BOOST ENGAGED (100%) | Motion: {self.last_cmd:<12}\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                if ch in ['1', '2', '3']:
                    gear_map = {
                        '1': (0.55, 'GEAR_1', '1 (55%)'),
                        '2': (0.75, 'GEAR_2', '2 (75%)'),
                        '3': (1.00, 'GEAR_3', '3 (100%)')
                    }
                    self.base_gear_multiplier, self.base_gear_cmd, self.base_gear_label = gear_map[ch]
                    
                    # If not currently holding shift, apply the gear immediately
                    if not self.is_boosted:
                        self.send_gear_update(self.base_gear_cmd)
                        sys.stdout.write(f"\r>>> GEAR SWITCHED: {self.base_gear_label:<10} | Motion: {self.last_cmd:<12}\n")
                        sys.stdout.flush()
                else:
                    self.pressed_keys.add(ch)
        except AttributeError:
            pass

    def on_release(self, key):
        # Handle Shift (Boost) Release
        if key in [keyboard.Key.shift, keyboard.Key.shift_r]:
            if self.is_boosted:
                self.is_boosted = False
                self.send_gear_update(self.base_gear_cmd)
                sys.stdout.write(f"\r>>> BOOST RELEASED -> {self.base_gear_label:<10} | Motion: {self.last_cmd:<12}\n")
                sys.stdout.flush()
            return

        try:
            if hasattr(key, 'char') and key.char:
                ch = key.char.lower()
                self.pressed_keys.discard(ch)
        except AttributeError:
            pass

    def send_gear_update(self, gear_cmd: str):
        if gear_cmd != self.last_sent_gear_cmd:
            msg = String()
            msg.data = gear_cmd
            self.cmd_pub.publish(msg)
            self.last_sent_gear_cmd = gear_cmd

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
        
        # 1. Send hardware motion string to ESP32
        cmd_msg = String()
        cmd_msg.data = current_cmd
        self.cmd_pub.publish(cmd_msg)

        # 2. Determine effective scale (1.0 if boosted, else base gear)
        effective_scale = 1.00 if self.is_boosted else self.base_gear_multiplier
        active_label = "⚡ BOOST (100%)" if self.is_boosted else self.base_gear_label

        # 3. Publish scaled Twist message directly to Gazebo
        twist = Twist()
        if current_cmd == "FORWARD":
            twist.linear.x = self.base_speed * effective_scale
        elif current_cmd == "REVERSE":
            twist.linear.x = -self.base_speed * effective_scale
        elif current_cmd == "SPIN_LEFT":
            twist.angular.z = self.spin_omega * effective_scale
        elif current_cmd == "SPIN_RIGHT":
            twist.angular.z = -self.spin_omega * effective_scale
        elif current_cmd == "FWD_LEFT":
            twist.linear.x = (self.base_speed * 0.65) * effective_scale
            twist.angular.z = self.turn_omega * effective_scale
        elif current_cmd == "FWD_RIGHT":
            twist.linear.x = (self.base_speed * 0.65) * effective_scale
            twist.angular.z = -self.turn_omega * effective_scale
        elif current_cmd == "REV_LEFT":
            twist.linear.x = -(self.base_speed * 0.65) * effective_scale
            twist.angular.z = -self.turn_omega * effective_scale
        elif current_cmd == "REV_RIGHT":
            twist.linear.x = -(self.base_speed * 0.65) * effective_scale
            twist.angular.z = self.turn_omega * effective_scale

        self.cmd_vel_pub.publish(twist)

        if current_cmd != self.last_cmd:
            sys.stdout.write(f"\rCurrent Command: {current_cmd:<12} | Gear: {active_label:<14}")
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
        stop_cmd = String()
        stop_cmd.data = 'STOP'
        teleop_node.cmd_pub.publish(stop_cmd)
        teleop_node.cmd_vel_pub.publish(Twist())
        
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()