import rclpy
import time
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GROUP_NAME = "ur_manipulator"
TIP_LINK = "tool0"
BASE_FRAME = "base_link"

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

HOVER_Z = 0.28
PROBE_Z = 0.05
MOVE_SECONDS = 8
PAUSE_SECONDS = 3

TOUR = [
    ("centre",        0.45,  0.00),
    ("near-right",    0.30, -0.15),
    ("far-right",     0.60, -0.15),
    ("far-left",      0.60,  0.15),
    ("near-left",     0.30,  0.15),
    ("centre again",  0.45,  0.00),
]
rclpy.init()
node = Node("corner_tour")

ik_client = node.create_client(GetPositionIK, "/compute_ik")
node.get_logger().info("waiting for /compute_ik ...")
ik_client.wait_for_service()

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)
time.sleep(2.0)
def solve_ik(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.w = 0.0

    request = GetPositionIK.Request()
    request.ik_request.group_name = GROUP_NAME
    request.ik_request.ik_link_name = TIP_LINK
    request.ik_request.pose_stamped = pose
    request.ik_request.avoid_collisions = True
    request.ik_request.timeout.sec = 1

    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.error_code.val != 1:
        return None

    solution = dict(zip(
        result.solution.joint_state.name,
        result.solution.joint_state.position,
    ))
    return [solution[name] for name in JOINT_NAMES]
def go_to(angles, seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = angles
    point.time_from_start.sec = seconds
    msg.points = [point]
    publisher.publish(msg)
    time.sleep(seconds + 1)
for label, x, y in TOUR:
    node.get_logger().info(f"--- {label}: x={x}, y={y} ---")

    hover = solve_ik(x, y, HOVER_Z)
    if hover is None:
        node.get_logger().info("  no IK at hover, skipping")
        continue
    node.get_logger().info("  moving to hover")
    go_to(hover, MOVE_SECONDS)
    time.sleep(PAUSE_SECONDS)

    low = solve_ik(x, y, PROBE_Z)
    if low is None:
        node.get_logger().info("  no IK at probe depth")
        continue
    node.get_logger().info("  descending")
    go_to(low, MOVE_SECONDS)
    time.sleep(PAUSE_SECONDS)

    node.get_logger().info("  lifting")
    go_to(hover, MOVE_SECONDS)
    time.sleep(PAUSE_SECONDS)

node.get_logger().info("tour complete")
rclpy.shutdown()
