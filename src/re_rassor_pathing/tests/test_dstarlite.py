import unittest
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty, Header
from re_rassor_interfaces.msg import ObstacleArray, ObstacleLocation, LocationStatus
import subprocess
import sys
import threading


def set_header(msg, node, frame_id="map"):
    """Helper to set header with current time"""
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def make_wall(x_start, x_end, y):
    """Create a horizontal wall of obstacles"""
    obstacles = []
    for x in range(x_start, x_end + 1):
        obstacles.append((x, y))
    return obstacles


def make_box(center_x, center_y, size):
    """Create a box perimeter of obstacles"""
    obstacles = []
    for dx in range(-size, size + 1):
        for dy in range(-size, size + 1):
            if abs(dx) == size or abs(dy) == size:
                obstacles.append((center_x + dx, center_y + dy))
    return obstacles


class RoutingTest(Node):
    def __init__(self):
        super().__init__('routing_test_node')

        self.received_path = None
        self.current_planner_status = None
        self.status_history = []
        self.path_count = 0
        self.last_path_time = None

        # Publishers
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/obstacle_array', 10)
        self.location_pub = self.create_publisher(
            LocationStatus, '/location_status', 10)
        self.replan_pub = self.create_publisher(
            Empty, '/replan', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/planning/path', self.path_callback, 10)
        self.planner_status_sub = self.create_subscription(
            String, '/planning/planner_status', self.planner_status_callback, 10)

    def path_callback(self, msg):
        """Callback for path messages from routing node"""
        self.path_count += 1
        self.last_path_time = time.time()
        self.get_logger().info(
            f"Received path #{self.path_count} with {len(msg.poses)} waypoints")
        self.received_path = msg
        
        # Print waypoints
        print(f"\n[Path #{self.path_count}] Waypoints:")
        for i, pose in enumerate(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            print(f"  [{i}] ({x:.2f}, {y:.2f}, {z:.2f})")
        print()

    def planner_status_callback(self, msg):
        """Callback for planner status messages"""
        self.current_planner_status = msg.data
        self.status_history.append((msg.data, time.time()))
        print(f"[Planner Status] {msg.data}")

    def reset_tracking(self):
        """Reset all tracking variables for new test scenarios"""
        self.received_path = None
        self.path_count = 0
        self.last_path_time = None
        self.status_history = []


class TestRoutingNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Start the routing node process"""
        cls.routing_process = subprocess.Popen(
            ["ros2", "run", "re_rassor_pathing", "routing_node"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        def print_stream(stream, prefix):
            for line in iter(stream.readline, ''):
                sys.stdout.write(f"[{prefix}] {line}")
            stream.close()

        cls.routing_stdout_thread = threading.Thread(
            target=print_stream, args=(cls.routing_process.stdout, "ROUTING-OUT"), daemon=True
        )
        cls.routing_stderr_thread = threading.Thread(
            target=print_stream, args=(cls.routing_process.stderr, "ROUTING-ERR"), daemon=True
        )

        cls.routing_stdout_thread.start()
        cls.routing_stderr_thread.start()

        time.sleep(3)

        rclpy.init()
        cls.node = RoutingTest()
    
    @classmethod
    def tearDownClass(cls):
        """Clean up processes and ROS"""
        cls.routing_process.terminate()
        cls.routing_process.wait(timeout=5)
        rclpy.shutdown()
    
    def create_obstacle_array(self, obstacles):
        """Create ObstacleArray message from list of (x, y) tuples"""
        msg = ObstacleArray()
        for x, y in obstacles:
            obs = ObstacleLocation()
            obs.x = float(x)
            obs.y = float(y)
            msg.obstacles.append(obs)
        return msg

    def create_location_status(self, pos_x, pos_y, goal_x, goal_y):
        """Create LocationStatus message"""
        msg = LocationStatus()
        msg.position_x = float(pos_x)
        msg.position_y = float(pos_y)
        msg.goal_x = float(goal_x)
        msg.goal_y = float(goal_y)
        msg.time = 0.0
        msg.velocity = 0.0
        msg.orientation = 0.0
        return msg

    def wait_for_subscribers(self, timeout=10):
        """Wait for routing node to subscribe to our topics"""
        start_time = time.time()
        while (self.node.obstacle_pub.get_subscription_count() == 0 or
               self.node.location_pub.get_subscription_count() == 0 or
               self.node.replan_pub.get_subscription_count() == 0) and \
              time.time() - start_time < timeout:
            print(f"Waiting for subscribers... {time.time() - start_time:.1f}s")
            rclpy.spin_once(self.node, timeout_sec=0.1)
        print("Topics subscribed!")

    def wait_for_planner_status(self, expected_status, timeout=10):
        """Wait for a specific planner status"""
        start_time = time.time()
        while self.node.current_planner_status != expected_status and \
              time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.current_planner_status == expected_status

    def wait_for_path(self, timeout=10):
        """Wait for a new path to be received"""
        initial_count = self.node.path_count
        start_time = time.time()
        while self.node.path_count <= initial_count and \
              time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.node.path_count > initial_count

    def test_basic_planning(self):
        """Test #1: Basic path planning from start to goal"""
        print("\n=== Test #1: Basic Planning ===")
        
        self.wait_for_subscribers()
        self.node.reset_tracking()

        # Send empty obstacle array
        obstacle_msg = self.create_obstacle_array([])
        self.node.obstacle_pub.publish(obstacle_msg)
        print("Sent empty obstacle grid")
        time.sleep(0.5)

        # Set start and goal
        location_msg = self.create_location_status(
            pos_x=10.0, pos_y=10.0, goal_x=50.0, goal_y=50.0)
        self.node.location_pub.publish(location_msg)
        print("Sent location (start: 10,10 | goal: 50,50)")
        time.sleep(0.5)

        # Trigger planning
        replan_msg = Empty()
        self.node.replan_pub.publish(replan_msg)
        print("Triggered planning")
        time.sleep(0.5)
        
        # Wait for path
        path_received = self.wait_for_path(timeout=10)
        
        self.assertTrue(path_received, "No path received from planner")
        self.assertIsNotNone(self.node.received_path, "Path is None")
        self.assertGreater(len(self.node.received_path.poses), 0, "Path is empty")

        print(f"✓ Basic planning successful: {len(self.node.received_path.poses)} waypoints")

    def test_planning_with_obstacles(self):
        """Test #2: Path planning with obstacles"""
        print("\n=== Test #2: Planning with Obstacles ===")
        
        self.wait_for_subscribers()
        self.node.reset_tracking()

        # Create wall of obstacles
        

        # Set start and goal
        location_msg = self.create_location_status(
            pos_x=10.0, pos_y=10.0, goal_x=50.0, goal_y=50.0)
        self.node.location_pub.publish(location_msg)
        print("Sent location (start: 10,10 | goal: 50,50)")
        time.sleep(0.5)

        obstacles = [(50,11)]
        obstacle_msg = self.create_obstacle_array(obstacles)
        self.node.obstacle_pub.publish(obstacle_msg)
        print(f"Sent obstacle wall with {len(obstacles)} obstacles")
        time.sleep(0.5)

        # Trigger planning
        replan_msg = Empty()
        self.node.replan_pub.publish(replan_msg)
        print("Triggered planning")
        time.sleep(0.5)
        
        # Wait for path
        path_received = self.wait_for_path(timeout=10)
        
        self.assertTrue(path_received, "No path received with obstacles")
        self.assertIsNotNone(self.node.received_path, "Path is None")
        self.assertGreater(len(self.node.received_path.poses), 0, "Path is empty")

        print(f"Planning with obstacles successful: {len(self.node.received_path.poses)} waypoints")

    def test_replanning(self):
        """Test #3: Dynamic replanning with new obstacles"""
        print("\n=== Test #3: Replanning ===")
        
        self.wait_for_subscribers()
        self.node.reset_tracking()

        # Initial planning without obstacles
        obstacle_msg = self.create_obstacle_array([])
        self.node.obstacle_pub.publish(obstacle_msg)
        print("Sent empty obstacle grid")
        time.sleep(0.5)

        location_msg = self.create_location_status(
            pos_x=10.0, pos_y=10.0, goal_x=50.0, goal_y=50.0)
        self.node.location_pub.publish(location_msg)
        print("Sent location (start: 10,10 | goal: 50,50)")
        time.sleep(0.5)

        replan_msg = Empty()
        self.node.replan_pub.publish(replan_msg)
        print("Triggered initial planning")
        time.sleep(0.5)

        path_received = self.wait_for_path(timeout=10)
        self.assertTrue(path_received, "No initial path received")
        initial_path_length = len(self.node.received_path.poses)
        print(f"Initial path: {initial_path_length} waypoints")

        # Reset and add obstacles
        self.node.reset_tracking()
        
        obstacles = [(11,10)]
        obstacle_msg = self.create_obstacle_array(obstacles)
        self.node.obstacle_pub.publish(obstacle_msg)
        print(f"Added {len(obstacles)} new obstacles")
        time.sleep(0.5)

        # Trigger replan
        self.node.replan_pub.publish(replan_msg)
        print("Triggered replanning")
        time.sleep(0.5)

        path_received = self.wait_for_path(timeout=10)
        
        self.assertTrue(path_received, "No replanned path received")
        self.assertIsNotNone(self.node.received_path, "Replanned path is None")
        self.assertGreater(len(self.node.received_path.poses), 0, "Replanned path is empty")

        print(f"✓ Replanning successful: {len(self.node.received_path.poses)} waypoints")

    def test_unreachable_goal(self):
        """Test #4: Handling unreachable goal"""
        print("\n=== Test #4: Unreachable Goal ===")
        
        self.wait_for_subscribers()
        self.node.reset_tracking()

        # Create box around goal
        obstacles = make_box(50, 50, 5)
        obstacle_msg = self.create_obstacle_array(obstacles)
        self.node.obstacle_pub.publish(obstacle_msg)
        print(f"Sent box of {len(obstacles)} obstacles around goal")
        time.sleep(0.5)

        # Try to reach blocked goal
        location_msg = self.create_location_status(
            pos_x=10.0, pos_y=10.0, goal_x=50.0, goal_y=50.0)
        self.node.location_pub.publish(location_msg)
        print("Sent location to blocked goal")
        time.sleep(0.5)

        replan_msg = Empty()
        self.node.replan_pub.publish(replan_msg)
        print("Triggered planning")
        time.sleep(0.5)

        # Should either get no path or a failure status
        path_received = self.wait_for_path(timeout=10)
        
        if path_received:
            print(f"Path received with {len(self.node.received_path.poses)} waypoints (may not reach goal)")
        else:
            print("No path received (goal is unreachable)")


if __name__ == '__main__':
    unittest.main(verbosity=2)