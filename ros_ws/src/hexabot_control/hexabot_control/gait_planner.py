import rclpy
from rclpy.node import Node
from enum import Enum, auto
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from gait_matching.py import neutral_tips, leg_phases, tip_trajectory, compute_tip_targets

class HexapodState(Enum):
    STOP = auto()
    REST = auto()
    IDLE = auto()
    TRIPOD_FORWARD = auto()
    TRIPOD_REVERSE = auto()
    WAVE_FORWARD = auto()
    WAVE_REVERSE = auto()
    RIPPLE_FORWARD = auto()
    RIPPLE_REVERSE = auto()

    TRIPOD_RIGHT = auto()
    TRIPOD_LEFT = auto()
    WAVE_RIGHT = auto()
    WAVE_LEFT = auto()
    RIPPLE_RIGHT = auto()
    RIPPLE_LEFT = auto()


class HexapodFSMNode(Node):
    def __init__(self):
        super().__init__('hexapod_fsm_node')

        # Internal FSM inputs (Boolean flags)
        self.move = False
        self.type = False       # True = Tripod (Dynamic), False = Wave (Static)
        self.direction = True   # True = Forward, False = Reverse

        # FSM State tracking
        self.current_state = HexapodState.STOP

        # Subscribers for boolean inputs
        self.create_subscription(
            Bool, 'hexapod/cmd_move', self.move_callback, 10
        )
        self.create_subscription(
            Bool, 'hexapod/cmd_type', self.type_callback, 10
        )
        self.create_subscription(
            Bool, 'hexapod/cmd_direction', self.direction_callback, 10
        )

        # Publishers for FSM status and velocity commands
        self.pub_state = self.create_publisher(String, 'hexapod/current_state', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Main control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Hexapod FSM Node started.')

    # Topic Callbacks
    def move_callback(self, msg: Bool):
        self.move = msg.data

    def type_callback(self, msg: Bool):
        self.type = msg.data

    def direction_callback(self, msg: Bool):
        self.direction = msg.data

    def update_fsm(self):
        if not self.move:
            self.current_state = HexapodState.IDLE
        elif self.type and self.direction:
            self.current_state = HexapodState.TRIPOD_FORWARD
        elif self.type and not self.direction:
            self.current_state = HexapodState.TRIPOD_REVERSE
        elif not self.type and self.direction:
            self.current_state = HexapodState.TURN_LEFT
        else:
            self.current_state = HexapodState.TURN_RIGHT

    def control_loop(self):
        self.update_fsm()

        state_msg = String()
        state_msg.data = self.current_state.name
        self.pub_state.publish(state_msg)

        cmd_vel = Twist()

        if self.current_state == HexapodState.IDLE:
            cmd_vel.linear.x = 0.0
            # Call leg standing/park function here

        elif self.current_state == HexapodState.TRIPOD_FORWARD:
            cmd_vel.linear.x = 0.5  # Dynamic fast forward speed
            # Trigger tripod gait sequence

        elif self.current_state == HexapodState.TRIPOD_REVERSE:
            cmd_vel.linear.x = -0.5  # Dynamic fast reverse speed
            # Trigger tripod reverse gait sequence

        elif self.current_state == HexapodState.TURN_LEFT:
            cmd_vel.linear.x = 0.2  # Static slow, stable forward speed
            # Trigger wave gait sequence

        elif self.current_state == HexapodState.TURN_RIGHT:
            cmd_vel.linear.x = -0.2  # Static slow reverse speed
            # Trigger wave reverse gait sequence

        self.pub_cmd_vel.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()