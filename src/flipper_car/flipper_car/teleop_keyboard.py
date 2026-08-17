#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

BANNER = """
---------------------------------------------
🎮 Flipper Car WASD + T Teleop
---------------------------------------------
Controls:
   W: Forward
   A: Rotate Full Left (Spin)
   D: Rotate Full Right (Spin)
   Q: Forward-Left
   E: Forward-Right
   S / Space: STOP
   
   T: Toggle Mechanical Polarity (Servo 0° / 180°)
   CTRL-C: Quit
---------------------------------------------
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub = self.create_publisher(String, '/flipper/command', 10)
        self.polarity_pub = self.create_publisher(Bool, '/flipper/polarity', 10)
        
        self.polarity_reverse = False  # False = 0 deg, True = 180 deg
        self.current_cmd = "STOP"
        
        print(BANNER)
        self.publish_status()

    def publish_status(self):
        pol_str = "REVERSE (180°)" if self.polarity_reverse else "FORWARD (0°)"
        sys.stdout.write(f"\rCurrent Command: {self.current_cmd:<12} | Polarity: {pol_str:<15}")
        sys.stdout.flush()

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)
                
                if key:
                    key_lower = key.lower()
                    if key_lower == 'w':
                        self.current_cmd = "FORWARD"
                    elif key_lower == 'a':
                        self.current_cmd = "FULL_LEFT"
                    elif key_lower == 'd':
                        self.current_cmd = "FULL_RIGHT"
                    elif key_lower == 'q':
                        self.current_cmd = "FWD_LEFT"
                    elif key_lower == 'e':
                        self.current_cmd = "FWD_RIGHT"
                    elif key_lower in ['s', ' ']:
                        self.current_cmd = "STOP"
                    elif key_lower == 't':
                        self.polarity_reverse = not self.polarity_reverse
                        pol_msg = Bool()
                        pol_msg.data = self.polarity_reverse
                        self.polarity_pub.publish(pol_msg)
                    elif key == '\x03':  # CTRL-C
                        break
                    
                    cmd_msg = String()
                    cmd_msg.data = self.current_cmd
                    self.cmd_pub.publish(cmd_msg)
                    self.publish_status()
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()