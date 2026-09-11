import rclpy
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

HANDSHAKE_WAIT = 2.0
STAY_ALIVE_WAIT = 10.0

rclpy.init()
node = Node("first_move")

publisher = node.create_publisher(
    JointTrajectory,
    "/scaled_joint_trajectory_controller/joint_trajectory",
    10,
)

time.sleep(HANDSHAKE_WAIT)

msg = JointTrajectory()
msg.joint_names = JOINT_NAMES

point_a = JointTrajectoryPoint()
point_a.positions = [0.5, -1.57, 0.0, -1.57, 0.0, 0.0]
point_a.time_from_start.sec = 4

point_b = JointTrajectoryPoint()
point_b.positions = [-0.5, -1.57, 0.0, -1.57, 0.0, 0.0]
point_b.time_from_start.sec = 8

msg.points = [point_a, point_b]

publisher.publish(msg)
node.get_logger().info("move command published")

time.sleep(STAY_ALIVE_WAIT)
rclpy.shutdown()
