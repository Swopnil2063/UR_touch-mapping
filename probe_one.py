import rclpy
import csv
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

POSE_FILE = "grid_poses.csv"
DESCENT_STEPS = 6

rclpy.init()
node = Node("probe_one")

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)

latest_fz = None
latest_joints = None

def wrench_callback(msg):
    global latest_fz
    latest_fz = msg.wrench.force.z

def joint_callback(msg):
    global latest_joints
    lookup = dict(zip(msg.name, msg.position))
    latest_joints = [lookup[n] for n in JOINT_NAMES]

node.create_subscription(
    WrenchStamped, "/force_torque_sensor_broadcaster/wrench", wrench_callback, 10
)
node.create_subscription(JointState, "/joint_states", joint_callback, 10)

def wait(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

wait(2.0)
node.get_logger().info(f"force.z = {latest_fz}")
node.get_logger().info(f"joints  = {latest_joints}")

rclpy.shutdown()
