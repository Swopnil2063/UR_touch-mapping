import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

GROUP_NAME = "ur_manipulator"
TIP_LINK = "tool0"
BASE_FRAME = "base_link"

X_VALUES = [0.30, 0.36, 0.42, 0.48, 0.54, 0.60]
Y_VALUES = [-0.15, -0.09, -0.03, 0.03, 0.09, 0.15]
HOVER_Z = 0.28

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
    # 180 degrees about x: makes the tool point straight down
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
    return future.result().error_code.val == 1

print(f"\nhover layer at z = {HOVER_Z}")
print("        " + "".join(f"{x:>7.2f}" for x in X_VALUES))
reachable = 0
for y in Y_VALUES:
    marks = ""
    for x in X_VALUES:
        if check(x, y, HOVER_Z):
            marks += "     ok"
            reachable += 1
        else:
            marks += "     XX"
    print(f"y={y:+.2f} {marks}")
print(f"\n{reachable} of 36 hover points reachable")

print("\ndescent depth at corners and centre:")
for (x, y) in [(0.30, -0.15), (0.60, -0.15), (0.30, 0.15), (0.60, 0.15), (0.45, 0.00)]:
    lowest = None
    for z in [0.28, 0.24, 0.20, 0.16, 0.12, 0.08, 0.04, 0.02]:
        if check(x, y, z):
            lowest = z
        else:
            break
    print(f"  ({x:+.2f}, {y:+.2f})  lowest z reached: {lowest}")

rclpy.shutdown()
