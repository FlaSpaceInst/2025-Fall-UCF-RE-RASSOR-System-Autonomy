import pytest
import rclpy
import time

from rclpy.node import Node
from re_rassor_interfaces.msg import LocationStatus   


'''
TEST DSTARLITE's PATHING

'''


class TestLocationStatusPathing(Node):

    def __init__(self):
        super().__init__("test_location_status_pathing")
        self.received_msg = None

        self.subscription = self.create_subscription(
            LocationStatus,
            "/pathing/location_status_out", 
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            LocationStatus,
            "/pathing/location_status_in",   
            10
        )

    def callback(self, msg):
        self.received_msg = msg


@pytest.fixture(scope="module")
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_location_status_processing(ros_init):
    node = TestLocationStatusPathing()

    msg = LocationStatus()
    msg.position_x = 1.5
    msg.position_y = -2.3
    msg.time = 3.0
    msg.velocity = 0.45
    msg.orientation = 1.57
    msg.goal_x = 4.0
    msg.goal_y = 8.0

    node.publisher.publish(msg)

    timeout = time.time() + 3.0
    while node.received_msg is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Validate received message
    assert node.received_msg is not None, \
        "No LocationStatus message received from pathing node!"

    
    assert node.received_msg.goal_x == 4.0
    assert node.received_msg.goal_y == 8.0
    assert abs(node.received_msg.position_x - 1.5) < 1e-6
    assert abs(node.received_msg.position_y + 2.3) < 1e-6


def teardown_module():
    rclpy.shutdown()
