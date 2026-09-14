import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

GROUP_NAME = "ur_manipulator"
TIP_LINK = "tool0"
BASE_FRAME = "base_link"

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

X_VALUES = [0.30, 0.40, 0.50, 0.60]
Y_VALUES = [-0.15, -0.05, 0.05, 0.15]

FINGER_LENGTH = 0.09
HOVER_Z = 0.30 + FINGER_LENGTH
rclpy.init()
node = Node("ik_check")

client = node.create_client(GetPositionIK, "/compute_ik")
node.get_logger().info("waiting for /compute_ik ...")
client.wait_for_service()

def check(x, y, z):
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

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    if result.error_code.val != 1:
        return None
    sol = dict(zip(result.solution.joint_state.name,
                   result.solution.joint_state.position))
    return [sol[n] for n in JOINT_NAMES]
print(f"\nhover layer, tool0 at z = {HOVER_Z:.2f}")
print("                  pan     lift    elbow   wr1     wr2     wr3")
ok_count = 0
for y in Y_VALUES:
    for x in X_VALUES:
        angles = check(x, y, HOVER_Z)
        if angles is None:
            print(f"x={x:.2f} y={y:+.2f}   FAILED")
        else:
            ok_count += 1
            formatted = " ".join(f"{a:+.3f}" for a in angles)
            print(f"x={x:.2f} y={y:+.2f}   {formatted}")

print(f"\n{ok_count} of 16 hover points reachable")

print("\ndescent depth check:")
for (x, y) in [(0.30, -0.15), (0.60, -0.15), (0.30, 0.15), (0.60, 0.15), (0.45, 0.00)]:
    lowest = None
    for z in [0.39, 0.30, 0.25, 0.20, 0.15, 0.12, 0.09]:
        if check(x, y, z) is not None:
            lowest = z
        else:
            break
    print(f"  ({x:+.2f}, {y:+.2f})  lowest tool0 z: {lowest}")

rclpy.shutdown()

