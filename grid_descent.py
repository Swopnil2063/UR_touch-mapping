import rclpy
import csv
import time
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

START_POSE = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
POSE_FILE = "grid_poses.csv"
LOG_FILE = "descent_log.csv"

DESCENT_STEPS = 6
REPOSITION_SECONDS = 8
DESCEND_SECONDS = 20
LIFT_SECONDS = 3
rclpy.init()
node = Node("grid_descent")

publisher = node.create_publisher(
    JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
)

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
time.sleep(2.0)

def wait(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

def publish_single(angles, seconds):
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
run_start = time.time()

publish_single(START_POSE, 4)
wait(5)

log = []
for index, (gx, gy, column) in enumerate(points, start=1):
    publish_single(column[0], REPOSITION_SECONDS)
    wait(REPOSITION_SECONDS + 0.3)

    publish_column(column, DESCEND_SECONDS)
    wait(DESCEND_SECONDS + 0.5)

    pos = tip_position()
    if pos is None:
        node.get_logger().info(f"[{index}/16] x={gx:.2f} y={gy:+.2f}  no reading")
    else:
        px, py, pz = pos
        ex = abs(px - gx)
        ey = abs(py - gy)
        node.get_logger().info(
            f"[{index}/16] x={gx:.2f} y={gy:+.2f}  ->  "
            f"({px:+.3f}, {py:+.3f}, {pz:+.3f})  err=({ex:.3f}, {ey:.3f})"
        )
        log.append([gx, gy, px, py, pz, ex, ey])

    publish_single(column[0], LIFT_SECONDS)
    wait(LIFT_SECONDS + 0.3)

publish_single(START_POSE, 4)
wait(5)

with open(LOG_FILE, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["grid_x", "grid_y", "actual_x", "actual_y", "actual_z", "err_x", "err_y"])
    w.writerows(log)

elapsed = time.time() - run_start
node.get_logger().info(f"done in {elapsed:.0f}s, {len(log)} rows in {LOG_FILE}")
rclpy.shutdown()

