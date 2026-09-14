import rclpy
import csv
import math
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

GROUP_NAME = "ur_manipulator"
TIP_LINK = "tool0"
BASE_FRAME = "base_link"

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

START_POSE = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

X_VALUES = [0.35, 0.42, 0.49, 0.56]
Y_VALUES = [-0.12, -0.04, 0.04, 0.12]
CENTER_X = 0.455
CENTER_Y = 0.0

FINGER_LENGTH = 0.09
HOVER_Z = 0.30
PROBE_Z = 0.00
DESCENT_STEPS = 6
MAX_JOINT_JUMP = 1.0
MAX_FIRST_JUMP = 2.5

OUTPUT_FILE = "grid_poses.csv"
rclpy.init()
node = Node("solve_grid")

client = node.create_client(GetPositionIK, "/compute_ik")
node.get_logger().info("waiting for /compute_ik ...")
client.wait_for_service()

def solve(x, y, tip_z, seed_angles, max_jump):
    pose = PoseStamped()
    pose.header.frame_id = BASE_FRAME
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = tip_z + FINGER_LENGTH
    pose.pose.orientation.x = 1.0
    pose.pose.orientation.w = 0.0

    seed_state = JointState()
    seed_state.name = list(JOINT_NAMES)
    seed_state.position = list(seed_angles)

    robot_state = RobotState()
    robot_state.joint_state = seed_state

    request = GetPositionIK.Request()
    request.ik_request.group_name = GROUP_NAME
    request.ik_request.ik_link_name = TIP_LINK
    request.ik_request.pose_stamped = pose
    request.ik_request.robot_state = robot_state
    request.ik_request.avoid_collisions = True
    request.ik_request.timeout.sec = 1

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.error_code.val != 1:
        return None

    sol = dict(zip(result.solution.joint_state.name,
                   result.solution.joint_state.position))
    raw = [sol[n] for n in JOINT_NAMES]

    unwrapped = []
    for angle, seed_angle in zip(raw, seed_angles):
        while angle - seed_angle > math.pi:
            angle -= 2 * math.pi
        while seed_angle - angle > math.pi:
            angle += 2 * math.pi
        unwrapped.append(angle)

    biggest_jump = max(abs(a - s) for a, s in zip(unwrapped, seed_angles))
    if biggest_jump > max_jump:
        return None

    return unwrapped
node.get_logger().info("solving reference pose at grid center")
reference = solve(CENTER_X, CENTER_Y, HOVER_Z, START_POSE, max_jump=10.0)
if reference is None:
    node.get_logger().info("reference pose failed - aborting")
    rclpy.shutdown()
    raise SystemExit
print("reference:", [f"{a:+.3f}" for a in reference])

step_heights = [HOVER_Z - (HOVER_Z - PROBE_Z) * i / (DESCENT_STEPS - 1)
                for i in range(DESCENT_STEPS)]
print("descent heights:", [f"{z:.3f}" for z in step_heights])

rows = []
failures = 0

for y in Y_VALUES:
    for x in X_VALUES:
        column = []
        current_seed = reference
        failed_at = None
        for step, z in enumerate(step_heights):
            limit = MAX_FIRST_JUMP if step == 0 else MAX_JOINT_JUMP
            angles = solve(x, y, z, current_seed, limit)
            if angles is None:
                failed_at = step
                break
            column.append(angles)
            current_seed = angles
        if failed_at is not None:
            print(f"x={x:.2f} y={y:+.2f}   FAILED at step {failed_at}")
            failures += 1
            continue
        flat = []
        for angles in column:
            flat.extend(angles)
        rows.append([x, y] + flat)
        lift = column[-1][1]
        elbow = column[-1][2]
        print(f"x={x:.2f} y={y:+.2f}  ok   bottom lift={lift:+.3f} elbow={elbow:+.3f}")

print(f"\n{len(rows)} points solved, {failures} failed")

header = ["x", "y"]
for step in range(DESCENT_STEPS):
    header += [f"s{step}_{n}" for n in JOINT_NAMES]

with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(header)
    writer.writerows(rows)

print(f"written to {OUTPUT_FILE}")
rclpy.shutdown()

