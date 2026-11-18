import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, UInt8

from hexabot_control.config import (
    MODE_ESTOP, MODE_SIT, MODE_STAND, MODE_WALK, NUM_LEGS, load_geometry)
from hexabot_control.gait_matching import GAITS, compute_tip_targets, neutral_tips
from hexabot_control.limits import clamp, slew
from hexabot_interfaces.msg import TipTargets
from hexabot_interfaces.srv import SetGait, SetMode


class BodyController(Node):

    def __init__(self):
        super().__init__('body_controller')
        d = self.declare_parameter
        self.geom = load_geometry(self)

        self.stance_radius = d('stance_radius', 0.11).value
        self.stand_height = d('stand_height', 0.06).value
        self.sit_height = d('sit_height', 0.02).value
        self.cycle_time = d('cycle_time', 1.0).value
        self.step_height = d('step_height', 0.03).value
        gait_name = d('gait', 'tripod').value

        self.max_vx = d('max_vx', 0.06).value
        self.max_vy = d('max_vy', 0.06).value
        self.max_wz = d('max_wz', 0.5).value
        self.accel = d('accel', 0.2).value
        self.height_rate = d('height_rate', 0.05).value
        self.cmd_timeout = d('cmd_timeout', 0.3).value
        self.dt = 1.0 / d('rate', 50.0).value

        if gait_name not in GAITS:
            raise ValueError(f'gait must be one of {list(GAITS)}')
        self.gait = GAITS[gait_name]
        self.pending_gait = None

        self.mode = d('initial_mode', MODE_SIT).value
        self.neutral_xy = neutral_tips(self.geom, self.stance_radius)

        self.phase = 0.0
        self.height = self.sit_height
        self.vel = np.zeros(3)
        self.cmd = np.zeros(3)
        self.last_cmd = None

        self.pub = self.create_publisher(TipTargets, 'tip_targets', qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, 'gait_cmd', self._on_gait, 10)
        self.create_subscription(UInt8, 'mode_cmd', self._on_mode, 10)
        self.create_service(SetMode, 'set_mode', self._srv_set_mode)
        self.create_service(SetGait, 'set_gait', self._srv_set_gait)

        self.create_timer(self.dt, self._tick)
        self.get_logger().info('body_controller started')

    def _on_cmd_vel(self, msg):
        self.cmd = np.array([
            clamp(msg.linear.x, -self.max_vx, self.max_vx),
            clamp(msg.linear.y, -self.max_vy, self.max_vy),
            clamp(msg.angular.z, -self.max_wz, self.max_wz)])
        self.last_cmd = self.get_clock().now()

    def _on_gait(self, msg):
        self._request_gait(msg.data)

    def _on_mode(self, msg):
        self._set_mode(msg.data)

    def _request_gait(self, name):
        if name not in GAITS:
            self.get_logger().warn(f'Unknown gait "{name}"')
            return False
        self.pending_gait = GAITS[name]
        return True

    def _set_mode(self, mode):
        if mode not in (MODE_SIT, MODE_STAND, MODE_WALK, MODE_ESTOP):
            self.get_logger().warn(f'Unknown mode {mode}')
            return False
        self.mode = mode
        if mode == MODE_ESTOP:
            self.vel[:] = 0.0
            self.cmd[:] = 0.0
        return True

    def _srv_set_mode(self, request, response):
        response.success = self._set_mode(request.mode)
        response.message = 'ok' if response.success else 'invalid mode'
        return response

    def _srv_set_gait(self, request, response):
        if request.cycle_time <= 0.0 or request.step_height < 0.0:
            response.success = False
            response.message = 'cycle_time must be > 0 and step_height >= 0'
            return response
        response.success = self._request_gait(request.gait_type)
        response.message = 'ok' if response.success else 'unknown gait'
        if response.success:
            self.cycle_time = float(request.cycle_time)
            self.step_height = float(request.step_height)
        return response

    def _tick(self):
        if self.mode == MODE_ESTOP:
            return

        target_height = self.sit_height if self.mode == MODE_SIT else self.stand_height
        self.height = slew(self.height, target_height, self.height_rate * self.dt)
        standing = abs(self.height - self.stand_height) < 0.005

        target = np.zeros(3)
        if self.mode == MODE_WALK and standing and self.last_cmd is not None:
            age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
            if age <= self.cmd_timeout:
                target = self.cmd

        step = self.accel * self.dt
        self.vel = np.array([slew(self.vel[i], target[i], step) for i in range(3)])

        moving = np.linalg.norm(self.vel) > 1e-6
        if moving:
            self.phase = (self.phase + self.dt / self.cycle_time) % 1.0
        else:
            self.phase = 0.0
            if self.pending_gait is not None:
                self.gait = self.pending_gait
                self.pending_gait = None
                self.get_logger().info('Gait changed')

        hold = np.full(NUM_LEGS, not moving)
        positions, contact, phases = compute_tip_targets(
            self.phase, self.gait, self.cycle_time, self.step_height, self.neutral_xy,
            self.vel, self.height, np.eye(3), (0.0, 0.0), hold)

        msg = TipTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.positions = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in positions]
        msg.contact = [bool(c) for c in contact]
        msg.phase = [float(p) for p in phases]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BodyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
