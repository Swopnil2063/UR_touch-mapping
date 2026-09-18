import rclpy
import csv
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

START_POSE = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
POSE_FILE = "grid_poses_2.csv"
LOG_FILE = "touch_map_log_2.csv"

# ---- TUNE THESE WITHOUT RE-PULLING ----
FORCE_THRESHOLD = 1.0      # newtons below baseline = contact
BASELINE_SECONDS = 1.0     # how long to average free-air force
DESCENT_STEPS = 6
REPOSITION_SECONDS = 8
DESCEND_SECONDS = 20
LIFT_SECONDS = 3
DRY_RUN = False            # True = detect and report, never stop the arm
# ---------------------------------------

rclpy.init()
node = Node("touch_map")

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

latest_fz = None
latest_joints = None

def wrench_callback(msg):
    global latest_fz
    latest_fz = msg.wrench.force.z

def joint_callback(msg):
    global latest_joints
    lookup = dict(zip(msg.name, msg.position))
    try:
        latest_joints = [lookup[n] for n in JOINT_NAMES]
    except KeyError:
        pass

node.create_subscription(
    WrenchStamped, "/force_torque_sensor_broadcaster/wrench", wrench_callback, 10
)
node.create_subscription(JointState, "/joint_states", joint_callback, 10)

def wait(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

def publish_single(angles, seconds):
    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = [float(a) for a in angles]
    pt.time_from_start.sec = int(seconds)
    pt.time_from_start.nanosec = int((seconds % 1) * 1e9)
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

def tip_position():
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = tf_buffer.lookup_transform("base_link", "tool0",
                                           rclpy.time.Time())
            p = t.transform.translation
            return p.x, p.y, p.z
        except Exception:
            continue
    return None

def measure_baseline():
    samples = []
    end = time.time() + BASELINE_SECONDS
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if latest_fz is not None:
            samples.append(latest_fz)
    if not samples:
        return None
    return sum(samples) / len(samples)

def freeze():
    """Stop the arm by commanding it to hold where it currently is."""
    if latest_joints is None:
        return
    publish_single(latest_joints, 0.1)

def descend_until_contact(column):
    """Send the full descent, watch force, cut it short on contact.
    Returns (contacted, baseline, force_at_contact)."""
    baseline = measure_baseline()
    if baseline is None:
        node.get_logger().error("no force data - aborting descent")
        return False, None, None

    publish_column(column, DESCENT_SECONDS_SAFE := DESCEND_SECONDS)

    deadline = time.time() + DESCEND_SECONDS + 0.5
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if latest_fz is None:
            continue
        drop = baseline - latest_fz
        if drop > FORCE_THRESHOLD:
            if not DRY_RUN:
                freeze()
            return True, baseline, latest_fz
    return False, baseline, latest_fz

# ---------- startup safety check ----------
wait(2.0)
node.get_logger().info(f"force.z = {latest_fz}")
node.get_logger().info(f"joints  = {latest_joints}")
if latest_fz is None or latest_joints is None:
    node.get_logger().error("missing sensor data - not moving. check driver.")
    rclpy.shutdown()
    raise SystemExit

# ---------- load grid ----------
points = []
with open(POSE_FILE, newline="") as f:
    for row in csv.DictReader(f):
        column = []
        for step in range(DESCENT_STEPS):
            column.append([row[f"s{step}_{n}"] for n in JOINT_NAMES])
        points.append((float(row["x"]), float(row["y"]), column))

node.get_logger().info(f"loaded {len(points)} points")
run_start = time.time()

publish_single(START_POSE, 4)
wait(5)

log = []
total = len(points)
for index, (gx, gy, column) in enumerate(points, start=1):
    publish_single(column[0], REPOSITION_SECONDS)
    wait(REPOSITION_SECONDS + 0.3)

    contacted, baseline, fz = descend_until_contact(column)
    wait(0.3)

    pos = tip_position()
    if pos is None:
        node.get_logger().info(f"[{index}/{total}] x={gx:.2f} y={gy:+.2f}  no tf reading")
    else:
        px, py, pz = pos
        tag = "CONTACT" if contacted else "no contact"
        node.get_logger().info(
            f"[{index}/{total}] x={gx:.2f} y={gy:+.2f}  {tag}  "
            f"z={pz:+.3f}  base={baseline:+.2f} fz={fz:+.2f}"
        )
        log.append([gx, gy, px, py, pz, int(contacted), baseline, fz])

    publish_single(column[0], LIFT_SECONDS)
    wait(LIFT_SECONDS + 0.3)

publish_single(START_POSE, 4)
wait(5)

with open(LOG_FILE, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["grid_x", "grid_y", "actual_x", "actual_y", "contact_z",
                "contacted", "baseline_fz", "fz_at_stop"])
    w.writerows(log)

elapsed = time.time() - run_start
hits = sum(r[5] for r in log)
node.get_logger().info(
    f"done in {elapsed:.0f}s, {len(log)} rows, {hits} contacts -> {LOG_FILE}"
)
rclpy.shutdown()
