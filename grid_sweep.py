import rclpy
import time
import csv
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

GROUP_NAME = "ur_manipulator"
TIP_LINK = "tool0"
BASE_FRAME = "base_link"

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

X_VALUES = [0.30, 0.42, 0.54, 0.66]
Y_VALUES = [-0.18, -0.06, 0.06, 0.18]
HOVER_Z = 0.28
PROBE_Z = 0.02

THRESHOLD = 20.0
MOVE_SECONDS = 6
DESCEND_SECONDS = 18
SETTLE_SECONDS = 1
DESCEND_TIMEOUT = 22.0

OUTPUT_FILE = "grid_results.csv"
rclpy.init()
node = Node("grid_sweep")

ik_client = node.create_client(GetPositionIK, "/compute_ik")
ik_client.wait_for_service()

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

latest_joint_positions = None
contact_detected = False

def on_joint_state(msg):
    global latest_joint_positions
    lookup = dict(zip(msg.name, msg.position))
    latest_joint_positions = [lookup[name] for name in JOINT_NAMES]

def on_force(msg):
    global contact_detected
    if msg.data > THRESHOLD:
        contact_detected = True

node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
node.create_subscription(Float64, "/fake_wrist_force", on_force, 10)
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

def publish_trajectory(angles, seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = angles
    point.time_from_start.sec = seconds
    msg.points = [point]
    publisher.publish(msg)

def go_to_and_wait(angles, seconds):
    publish_trajectory(angles, seconds)
    end_time = time.time() + seconds
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.05)

def get_tool_position():
    try:
        t = tf_buffer.lookup_transform(BASE_FRAME, TIP_LINK, rclpy.time.Time())
        p = t.transform.translation
        return p.x, p.y, p.z
    except Exception:
        return None
def probe_point(x, y):
    global contact_detected
    hover_angles = solve_ik(x, y, HOVER_Z)
    if hover_angles is None:
        return None
    go_to_and_wait(hover_angles, MOVE_SECONDS)

    low_angles = solve_ik(x, y, PROBE_Z)
    if low_angles is None:
        return None

    contact_detected = False
    publish_trajectory(low_angles, DESCEND_SECONDS)

    start = time.time()
    while time.time() - start < DESCEND_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.05)
        if contact_detected:
            break

    if latest_joint_positions is not None:
        publish_trajectory(latest_joint_positions, 0)
    time.sleep(SETTLE_SECONDS)

    result = get_tool_position()

    go_to_and_wait(hover_angles, MOVE_SECONDS)
    return result
results = []
for y in Y_VALUES:
    for x in X_VALUES:
        node.get_logger().info(f"probing x={x:.2f} y={y:.2f}")
        outcome = probe_point(x, y)
        if outcome is None:
            node.get_logger().info("  skipped (no IK solution)")
            continue
        px, py, pz = outcome
        node.get_logger().info(f"  contact at z={pz:.3f}")
        results.append((px, py, pz))

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z"])
    writer.writerows(results)

node.get_logger().info(f"done, {len(results)} points written to {OUTPUT_FILE}")
rclpy.shutdown()
