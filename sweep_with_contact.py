import rclpy
import csv
import time
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

START_POSE = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
POSE_FILE = "grid_poses.csv"
RESULT_FILE = "contact_results.csv"

DESCENT_STEPS = 6
THRESHOLD = 20.0
MOVE_SECONDS = 5
DESCEND_SECONDS = 14
DESCEND_TIMEOUT = 20.0
LIFT_SECONDS = 5
rclpy.init()
node = Node("sweep_with_contact")

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

latest_joints = None
contact_detected = False

def on_joint_state(msg):
    global latest_joints
    lookup = dict(zip(msg.name, msg.position))
    latest_joints = [lookup[n] for n in JOINT_NAMES]

def on_force(msg):
    global contact_detected
    if msg.data > THRESHOLD:
        contact_detected = True

node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
node.create_subscription(Float64, "/fake_wrist_force", on_force, 10)
time.sleep(2.0)
def publish_angles(angles, seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = [float(a) for a in angles]
    pt.time_from_start.sec = seconds
    msg.points = [pt]
    publisher.publish(msg)

def publish_column(column, total_seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    per_step = total_seconds / len(column)
    for i, angles in enumerate(column, start=1):
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in angles]
        elapsed = per_step * i
        pt.time_from_start.sec = int(elapsed)
        pt.time_from_start.nanosec = int((elapsed % 1) * 1e9)
        msg.points.append(pt)
    publisher.publish(msg)

def move_and_wait(angles, seconds):
    publish_angles(angles, seconds)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

def tip_position():
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = tf_buffer.lookup_transform("base_link", "probe_contact_point",
                                           rclpy.time.Time())
            p = t.transform.translation
            return p.x, p.y, p.z
        except Exception:
            continue
    return None
points = []
with open(POSE_FILE, newline="") as f:
    for row in csv.DictReader(f):
        column = []
        for step in range(DESCENT_STEPS):
            column.append([row[f"s{step}_{n}"] for n in JOINT_NAMES])
        points.append((float(row["x"]), float(row["y"]), column))

node.get_logger().info(f"loaded {len(points)} points")
node.get_logger().info("moving to start pose")
move_and_wait(START_POSE, MOVE_SECONDS)

results = []
for index, (gx, gy, column) in enumerate(points, start=1):
    node.get_logger().info(f"[{index}/{len(points)}] grid x={gx:.2f} y={gy:+.2f}")
    move_and_wait(column[0], MOVE_SECONDS)

    contact_detected = False
    publish_column(column, DESCEND_SECONDS)

    start = time.time()
    while time.time() - start < DESCEND_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.05)
        if contact_detected:
            break

    if latest_joints is not None:
        publish_angles(latest_joints, 0)
    time.sleep(0.5)

    pos = tip_position()
    if pos is None:
        node.get_logger().info("   no position reading")
    else:
        px, py, pz = pos
        hit = "CONTACT" if contact_detected else "no contact"
        node.get_logger().info(f"   {hit} at z={pz:.3f}")
        results.append([gx, gy, px, py, pz, int(contact_detected)])

    move_and_wait(column[0], LIFT_SECONDS)

move_and_wait(START_POSE, MOVE_SECONDS)

with open(RESULT_FILE, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["grid_x", "grid_y", "actual_x", "actual_y", "contact_z", "contacted"])
    w.writerows(results)

node.get_logger().info(f"done, {len(results)} rows written to {RESULT_FILE}")
rclpy.shutdown()

