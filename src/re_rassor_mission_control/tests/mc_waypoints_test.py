from re_rassor_interfaces.msg import ObstacleLocation
from re_rassor_interfaces.msg import ObstacleArray as ObstacleArrayMsg
from re_rassor_interfaces.srv import LocationStatus as LocationStatusMsg
from re_rassor_interfaces.srv import ObstacleArray as ObstacleArraySrv
from re_rassor_interfaces.srv import LocationStatus as LocationStatusSrv
from re_rassor_interfaces.srv import NewGoal
from re_rassor_interfaces.srv import FinalLocation 
from re_rassor_interfaces.srv import InitialRoverPosition
from re_rassor_interfaces.srv import InitialObstacleArray


import subprocess

import unittest
from rclpy.node import Node


class MissionControlTestNode(Node):
    def __init__(self):
        super().__init__('mc_test_node')
        

        #publishers here

        #subscribers here

    


def MissionControlTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.pathing_process = subprocess.Popen(
            ["ros2", "run", "re_rassor_mission_control", "re_rassor_mission_control"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests."""
        rclpy.shutdown()

    def test_waypoints(self):
        pass

