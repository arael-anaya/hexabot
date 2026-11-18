import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from hexabot_control.config import MODE_ESTOP, MODE_SIT, MODE_STAND, MODE_WALK

HELP = """
Hexabot keyboard teleop
-----------------------
  w / s   forward / backward
  q / e   strafe left / right
  a / d   turn left / right
  space   stop

  1 / 2 / 3   tripod / ripple / wave gait
  z / x / c   sit / stand / walk mode
  p           emergency stop (press c to recover)

  Ctrl-C  quit
"""

GAIT_KEYS = {'1': 'tripod', '2': 'ripple', '3': 'wave'}
MODE_KEYS = {'z': MODE_SIT, 'x': MODE_STAND, 'c': MODE_WALK, 'p': MODE_ESTOP}


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        d = self.declare_parameter
        self.linear = d('linear_speed', 0.06).value
        self.strafe = d('strafe_speed', 0.06).value
        self.angular = d('angular_speed', 0.5).value
        self.key_timeout = d('key_timeout', 0.5).value
        self.rate = d('rate', 20.0).value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gait_pub = self.create_publisher(String, 'gait_cmd', 10)
        self.mode_pub = self.create_publisher(UInt8, 'mode_cmd', 10)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_motion_key = None

    def handle_key(self, key):
        now = self.get_clock().now()
        if key in ('w', 's', 'q', 'e', 'a', 'd'):
            self.vx = self.vy = self.wz = 0.0
            if key == 'w':
                self.vx = self.linear
            elif key == 's':
                self.vx = -self.linear
            elif key == 'q':
                self.vy = self.strafe
            elif key == 'e':
                self.vy = -self.strafe
            elif key == 'a':
                self.wz = self.angular
            elif key == 'd':
                self.wz = -self.angular
            self.last_motion_key = now
        elif key == ' ':
            self.vx = self.vy = self.wz = 0.0
            self.last_motion_key = None
        elif key in GAIT_KEYS:
            self.gait_pub.publish(String(data=GAIT_KEYS[key]))
            print(f'gait: {GAIT_KEYS[key]}')
        elif key in MODE_KEYS:
            if key == 'p':
                self.vx = self.vy = self.wz = 0.0
                self.last_motion_key = None
            self.mode_pub.publish(UInt8(data=MODE_KEYS[key]))
            print(f'mode: {key}')

    def publish_cmd(self):
        if self.last_motion_key is not None:
            age = (self.get_clock().now() - self.last_motion_key).nanoseconds * 1e-9
            if age > self.key_timeout:
                self.vx = self.vy = self.wz = 0.0
                self.last_motion_key = None
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.cmd_pub.publish(msg)


def main(args=None):
    if not sys.stdin.isatty():
        print('keyboard_teleop needs an interactive terminal (use docker exec -it)')
        return

    rclpy.init(args=args)
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        period = 1.0 / node.rate
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], period)
            if ready:
                key = sys.stdin.read(1).lower()
                node.handle_key(key)
            node.publish_cmd()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
