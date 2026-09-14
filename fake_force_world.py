import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

BOX_X_MIN, BOX_X_MAX = 0.35, 0.55
BOX_Y_MIN, BOX_Y_MAX = -0.125, 0.125
BOX_TOP_Z = 0.15
TABLE_Z = 0.0

BASELINE_FORCE = 2.0
NOISE = 1.5
CONTACT_FORCE = 40.0

PUBLISH_PERIOD = 0.05
rclpy.init()
node = Node("fake_force_world")

publisher = node.create_publisher(Float64, "/fake_wrist_force", 10)

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
def tool_position():
    try:
        t = tf_buffer.lookup_transform("base_link", "probe_contact_point", rclpy.time.Time())
        p = t.transform.translation
        return p.x, p.y, p.z
    except Exception:
        return None

def force_at(x, y, z):
    inside_box = (BOX_X_MIN <= x <= BOX_X_MAX) and (BOX_Y_MIN <= y <= BOX_Y_MAX)
    if inside_box and z <= BOX_TOP_Z:
        return CONTACT_FORCE
    if z <= TABLE_Z:
        return CONTACT_FORCE
    return BASELINE_FORCE + random.uniform(-NOISE, NOISE)
def publish_reading():
    position = tool_position()
    msg = Float64()
    if position is None:
        msg.data = BASELINE_FORCE
    else:
        msg.data = force_at(*position)
    publisher.publish(msg)

node.create_timer(PUBLISH_PERIOD, publish_reading)
rclpy.spin(node)
