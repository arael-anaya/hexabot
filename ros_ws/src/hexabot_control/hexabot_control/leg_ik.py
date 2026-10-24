import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from hexabot_control.config import load_geometry
from hexabot_control.kinematics import solve_all_legs

from hexabot_interfaces.msg import LegJointTargets, TipTargets

class LegIK(Node):

    def __init__(self):
        super().__init__('leg_ik')
        self.geom = load_geometry(self)

        self.pub = self.create_publisher(
            LegJointTargets, 'leg_joint_targets', qos_profile_sensor_data)
        
        self.create_subscription(
            TipTargets, 'tip_targets', self._on_tip_targets, qos_profile_sensor_data)

    def _on_tip_targets(self, msg):
        tips = np.array([[p.x,p.y,p.z] for p in msg.positions])
        if tips.shape != (6,3) or not np.all(np.isfinite(tips)):
            self.get_logger().warn("Bad tip_targers message dropped")
            return
        q, reachable = solve_all_legs(tips, self.geom)
        if not reachable.all():
            legs = [i + 1 for i in range(6) if not reachable[i]]
            self.get_logger().warn(
                f'Tip target out of reach on legs {legs}, clamped', throttle_duration_sec = 2.0)
            out = LegJointTargets()
            out.header = msg.header
            out.position =   q.tolist()
            out.reachable = [bool(r) for r in reachable]
            self.pub.publish(out)

def main(args = None):
    rclpy.init(args = args)
    node = LegIK()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
