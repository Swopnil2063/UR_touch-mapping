import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

PUBLISH_PERIOD = 0.1      # seconds between readings
CLIMB_PER_READING = 0.1   # how much the value grows each time

rclpy.init()
node = Node("fake_force")

publisher = node.create_publisher(Float64, "/fake_wrist_force", 10)

current_force = 0.0

def publish_reading():
    global current_force
    msg = Float64()
    msg.data = current_force
    publisher.publish(msg)
    current_force = current_force + CLIMB_PER_READING

node.create_timer(PUBLISH_PERIOD, publish_reading)

rclpy.spin(node)

