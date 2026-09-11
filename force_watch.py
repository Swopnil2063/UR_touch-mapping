import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

THRESHOLD = 20.0

rclpy.init()
node = Node("force_watch")

contact_detected = False

def on_force_reading(msg):
    global contact_detected
    if contact_detected:
        return
    if msg.data > THRESHOLD:
        contact_detected = True
        node.get_logger().info(f"CONTACT at force {msg.data}")

node.create_subscription(Float64, "/fake_wrist_force", on_force_reading, 10)

rclpy.spin(node)
