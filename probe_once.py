import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

THRESHOLD = 30
DESCEND_TO = 1.5        # where wrist_1 heads toward
DESCEND_SECONDS = 40     # slow on purpose

rclpy.init()
node = Node("probe_once")

latest_positions = None
contact_detected = False
descent_started = False

publisher = node.create_publisher(
    JointTrajectory,
    "/scaled_joint_trajectory_controller/joint_trajectory",
    10,
)

def positions_in_order(state_msg):
    """Pair each joint name with its value, then return them in JOINT_NAMES order."""
    lookup = dict(zip(state_msg.name, state_msg.position))
    return [lookup[name] for name in JOINT_NAMES]

def send_trajectory(positions, seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = seconds
    msg.points = [point]
    publisher.publish(msg)

def on_joint_state(msg):
    global latest_positions
    latest_positions = positions_in_order(msg)

def on_force_reading(msg):
    global contact_detected
    if contact_detected or not descent_started:
        return
    if msg.data > THRESHOLD:
        contact_detected = True
        stop_at = list(latest_positions)
        send_trajectory(stop_at, 0)
        node.get_logger().info(f"CONTACT at force {msg.data}")
        node.get_logger().info(f"elbow angle: {stop_at[2]}")

def start_descent():
    global descent_started
    if latest_positions is None:
        node.get_logger().info("waiting for joint states...")
        return
    target = list(latest_positions)
    target[2] = DESCEND_TO
    send_trajectory(target, DESCEND_SECONDS)
    descent_started = True
    node.get_logger().info("descent started")
    startup_timer.cancel()

node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
node.create_subscription(Float64, "/fake_wrist_force", on_force_reading, 10)
startup_timer = node.create_timer(1.0, start_descent)

rclpy.spin(node)

