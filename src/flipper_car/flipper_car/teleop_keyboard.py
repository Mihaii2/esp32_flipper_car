#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

BANNER = """
-------------------------------------------------------
🎮 Flipper Car Multi-Key Teleop (Hold Keys Concurrently)
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
   CTRL-C     : Quit
-------------------------------------------------------
"""

class MultiKeyTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(String, '/flipper/command', 10)
        
        # Track currently held keys
        self.pressed_keys = set()
        self.last_cmd = "STOP"
        
        print(BANNER)
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                self.pressed_keys.add(key.char.lower())
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                self.pressed_keys.discard(key.char.lower())
        except AttributeError:
            pass

    def resolve_command(self) -> str:
        w = 'w' in self.pressed_keys
        s = 's' in self.pressed_keys
        a = 'a' in self.pressed_keys
        d = 'd' in self.pressed_keys

        # Forward combinations
        if w and not s:
            if a and not d:
                return "FWD_LEFT"
            if d and not a:
                return "FWD_RIGHT"
            return "FORWARD"

        # Reverse combinations
        if s and not w:
            if a and not d:
                return "REV_LEFT"
            if d and not a:
                return "REV_RIGHT"
            return "REVERSE"

        # Pure in-place spins (when neither W nor S is pressed)
        if a and not d:
            return "SPIN_LEFT"
        if d and not a:
            return "SPIN_RIGHT"

        return "STOP"

    def update_and_publish(self):
        current_cmd = self.resolve_command()
        
        # Publish and print status
        msg = String()
        msg.data = current_cmd
        self.cmd_pub.publish(msg)

        if current_cmd != self.last_cmd:
            sys.stdout.write(f"\rCurrent Command: {current_cmd:<15}")
            sys.stdout.flush()
            self.last_cmd = current_cmd

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MultiKeyTeleop()

    # Start non-blocking keyboard listener
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
        listener.stop()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()