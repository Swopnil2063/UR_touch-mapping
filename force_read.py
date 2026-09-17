import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

rclpy.init()
node = Node("force_read")

latest_fz = None

def wrench_callback(msg):
    global latest_fz
    latest_fz = msg.wrench.force.z

node.create_subscription(
    WrenchStamped, "/force_torque_sensor_broadcaster/wrench", wrench_callback, 10
)

end = time.time() + 10
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info(f"force.z = {latest_fz}")
    time.sleep(0.2)

rclpy.shutdown()
