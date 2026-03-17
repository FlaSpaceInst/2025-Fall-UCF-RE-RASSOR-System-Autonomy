#!/usr/bin/env python3
"""
Test script for motor_controller
Launches the motor_controller node and publishes ROS2 messages to test the HTTP POST bridge
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int8
import time
import sys
import subprocess
import threading
import signal


class MotorControllerTester(Node):
    def __init__(self):
        super().__init__('motor_controller_tester')
        
        self.motor_controller_process = None
        self.output_thread = None
        self.running = True
       
        self.wheel_pub = self.create_publisher(
            Twist, '/ezrassor/wheel_instructions', 10)
        self.front_arm_pub = self.create_publisher(
            Float64, '/ezrassor/front_arm_instructions', 10)
        self.back_arm_pub = self.create_publisher(
            Float64, '/ezrassor/back_arm_instructions', 10)
        self.front_drum_pub = self.create_publisher(
            Float64, '/ezrassor/front_drum_instructions', 10)
        self.back_drum_pub = self.create_publisher(
            Float64, '/ezrassor/back_drum_instructions', 10)
        self.routine_pub = self.create_publisher(
            Int8, '/ezrassor/routine_actions', 10)
        
        self.get_logger().info('Motor Controller Tester initialized')
    
    def start_motor_controller(self):
        """Launch the motor_controller node and monitor its output"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('Starting motor_controller node...')
        self.get_logger().info('=' * 70)
        
        try:
            # Start the motor_controller node
            self.motor_controller_process = subprocess.Popen(
                ['ros2', 'run', 're_rassor_motor_controller', 'motor_controller'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            # Start thread to monitor output
            self.output_thread = threading.Thread(
                target=self._monitor_output,
                daemon=True
            )
            self.output_thread.start()
            
            # Wait for node to initialize
            self.get_logger().info('Waiting for motor_controller to start...')
            time.sleep(3.0)
            
            # Check if process is still running
            if self.motor_controller_process.poll() is not None:
                self.get_logger().error('motor_controller process terminated unexpectedly!')
                return False
            
            self.get_logger().info('✓ motor_controller process started')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start motor_controller: {e}')
            return False
    
    def _monitor_output(self):
        """Monitor and print motor_controller output in real-time"""
        while self.running and self.motor_controller_process:
            line = self.motor_controller_process.stdout.readline()
            if line:
                print(f"[motor_controller] {line.rstrip()}", flush=True)
            elif self.motor_controller_process.poll() is not None:
                break
    
    def verify_node_running(self):
        """Verify the motor_controller node is running"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Verifying motor_controller node...')
        self.get_logger().info('=' * 70)
        
        try:
            # Get list of active nodes
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            nodes = result.stdout.strip().split('\n')
            self.get_logger().info(f'Active ROS2 nodes: {nodes}')
            
            if '/motor_controller' in nodes:
                self.get_logger().info('✓ motor_controller node is running')
                return True
            else:
                self.get_logger().error('✗ motor_controller node NOT found in node list')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to verify node: {e}')
            return False
    
    def verify_topics(self):
        """Verify all expected topics exist"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Verifying topics...')
        self.get_logger().info('=' * 70)
        
        expected_topics = [
            '/ezrassor/wheel_instructions',
            '/ezrassor/front_arm_instructions',
            '/ezrassor/back_arm_instructions',
            '/ezrassor/front_drum_instructions',
            '/ezrassor/back_drum_instructions',
            '/ezrassor/routine_actions'
        ]
        
        try:
            # Get list of active topics
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            active_topics = result.stdout.strip().split('\n')
            
            all_found = True
            for topic in expected_topics:
                if topic in active_topics:
                    self.get_logger().info(f'Topic exists: {topic}')
                else:
                    self.get_logger().error(f'Topic NOT found: {topic}')
                    all_found = False
            
            if all_found:
                self.get_logger().info('All expected topics exist')
            else:
                self.get_logger().error('Some topics are missing')
            
            return all_found
            
        except Exception as e:
            self.get_logger().error(f'Failed to verify topics: {e}')
            return False
    
    def wait_for_connections(self):
        """Wait for publishers to connect to subscribers"""
        self.get_logger().info('')
        self.get_logger().info('Waiting for topic connections...')
        
        max_wait = 5.0
        start_time = time.time()
        
        while time.time() - start_time < max_wait:
            if (self.wheel_pub.get_subscription_count() > 0 and
                self.front_arm_pub.get_subscription_count() > 0 and
                self.back_arm_pub.get_subscription_count() > 0):
                self.get_logger().info('Publishers connected to motor_controller')
                return True
            time.sleep(0.1)
        
        self.get_logger().warn('Timeout waiting for all connections, proceeding anyway...')
        return False
    
    def stop_motor_controller(self):
        """Stop the motor_controller process"""
        if self.motor_controller_process:
            self.get_logger().info('')
            self.get_logger().info('Stopping motor_controller node...')
            self.running = False
            self.motor_controller_process.terminate()
            try:
                self.motor_controller_process.wait(timeout=3)
                self.get_logger().info('motor_controller stopped cleanly')
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Force killing motor_controller...')
                self.motor_controller_process.kill()
                self.motor_controller_process.wait()

    def test_wheel_forward(self, duration=2):
        """Test forward motion"""
        self.get_logger().info('TEST: Moving forward...')
        msg = Twist()
        msg.linear.x = 0.5  # 0.5 m/s forward
        msg.angular.z = 0.0
        self.wheel_pub.publish(msg)
        time.sleep(duration)

    def test_wheel_backward(self, duration=2):
        """Test backward motion"""
        self.get_logger().info('TEST: Moving backward...')
        msg = Twist()
        msg.linear.x = -0.5  # 0.5 m/s backward
        msg.angular.z = 0.0
        self.wheel_pub.publish(msg)
        time.sleep(duration)

    def test_wheel_turn_left(self, duration=2):
        """Test left turn"""
        self.get_logger().info('TEST: Turning left...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0  # 1.0 rad/s left
        self.wheel_pub.publish(msg)
        time.sleep(duration)

    def test_wheel_turn_right(self, duration=2):
        """Test right turn"""
        self.get_logger().info('TEST: Turning right...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -1.0  # 1.0 rad/s right
        self.wheel_pub.publish(msg)
        time.sleep(duration)

    def test_wheel_stop(self):
        """Test stop"""
        self.get_logger().info('TEST: Stopping wheels...')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.wheel_pub.publish(msg)
        time.sleep(0.5)

    def test_front_arm_raise(self, duration=1.5):
        """Test raising front arm"""
        self.get_logger().info('TEST: Raising front arm...')
        msg = Float64()
        msg.data = 1.0  # Raise
        self.front_arm_pub.publish(msg)
        time.sleep(duration)

    def test_front_arm_lower(self, duration=1.5):
        """Test lowering front arm"""
        self.get_logger().info('TEST: Lowering front arm...')
        msg = Float64()
        msg.data = -1.0  # Lower
        self.front_arm_pub.publish(msg)
        time.sleep(duration)

    def test_front_arm_stop(self):
        """Test stopping front arm"""
        self.get_logger().info('TEST: Stopping front arm...')
        msg = Float64()
        msg.data = 0.0  # Stop
        self.front_arm_pub.publish(msg)
        time.sleep(0.5)

    def test_back_arm_raise(self, duration=1.5):
        """Test raising back arm"""
        self.get_logger().info('TEST: Raising back arm...')
        msg = Float64()
        msg.data = 1.0  # Raise
        self.back_arm_pub.publish(msg)
        time.sleep(duration)

    def test_back_arm_lower(self, duration=1.5):
        """Test lowering back arm"""
        self.get_logger().info('TEST: Lowering back arm...')
        msg = Float64()
        msg.data = -1.0  # Lower
        self.back_arm_pub.publish(msg)
        time.sleep(duration)

    def test_front_drum_dig(self, duration=1.5):
        """Test front drum digging"""
        self.get_logger().info('TEST: Front drum digging...')
        msg = Float64()
        msg.data = 1.0  # Dig
        self.front_drum_pub.publish(msg)
        time.sleep(duration)

    def test_front_drum_dump(self, duration=1.5):
        """Test front drum dumping"""
        self.get_logger().info('TEST: Front drum dumping...')
        msg = Float64()
        msg.data = -1.0  # Dump
        self.front_drum_pub.publish(msg)
        time.sleep(duration)

    def test_front_drum_stop(self):
        """Test stopping front drum"""
        self.get_logger().info('TEST: Stopping front drum...')
        msg = Float64()
        msg.data = 0.0  # Stop
        self.front_drum_pub.publish(msg)
        time.sleep(0.5)

    def test_back_drum_dig(self, duration=1.5):
        """Test back drum digging"""
        self.get_logger().info('TEST: Back drum digging...')
        msg = Float64()
        msg.data = 1.0  # Dig
        self.back_drum_pub.publish(msg)
        time.sleep(duration)

    def test_back_drum_dump(self, duration=1.5):
        """Test back drum dumping"""
        self.get_logger().info('TEST: Back drum dumping...')
        msg = Float64()
        msg.data = -1.0  # Dump
        self.back_drum_pub.publish(msg)
        time.sleep(duration)

    def test_routine_auto_drive(self, duration=2):
        """Test auto drive routine"""
        self.get_logger().info('TEST: Auto drive routine...')
        msg = Int8()
        msg.data = 0b000001  # AUTO_DRIVE
        self.routine_pub.publish(msg)
        time.sleep(duration)

    def test_routine_auto_dig(self, duration=2):
        """Test auto dig routine"""
        self.get_logger().info('TEST: Auto dig routine...')
        msg = Int8()
        msg.data = 0b000010  # AUTO_DIG
        self.routine_pub.publish(msg)
        time.sleep(duration)

    def test_routine_stop(self):
        """Test stop routine"""
        self.get_logger().info('TEST: Stop routine...')
        msg = Int8()
        msg.data = 0b100000  # STOP
        self.routine_pub.publish(msg)
        time.sleep(0.5)

    def run_quick_test(self):
        """Run a quick smoke test"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Running QUICK TEST')
        self.get_logger().info('=' * 60)
        
        self.test_wheel_forward(1)
        self.test_wheel_stop()
        self.test_front_arm_raise(1)
        self.test_front_arm_stop()
        self.test_front_drum_dig(1)
        self.test_front_drum_stop()
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Quick test completed!')
        self.get_logger().info('=' * 60)

    def run_all_tests(self):
        """Run all test sequences"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting COMPREHENSIVE motor controller tests')
        self.get_logger().info('=' * 60)
        
        # Wheel tests
        self.get_logger().info('\n--- WHEEL TESTS ---')
        self.test_wheel_forward()
        self.test_wheel_stop()
        self.test_wheel_backward()
        self.test_wheel_stop()
        self.test_wheel_turn_left()
        self.test_wheel_stop()
        self.test_wheel_turn_right()
        self.test_wheel_stop()
        
        # Front arm tests
        self.get_logger().info('\n--- FRONT ARM TESTS ---')
        self.test_front_arm_raise()
        self.test_front_arm_stop()
        self.test_front_arm_lower()
        self.test_front_arm_stop()
        
        # Back arm tests
        self.get_logger().info('\n--- BACK ARM TESTS ---')
        self.test_back_arm_raise()
        self.test_front_arm_stop()
        self.test_back_arm_lower()
        self.test_front_arm_stop()
        
        # Front drum tests
        self.get_logger().info('\n--- FRONT DRUM TESTS ---')
        self.test_front_drum_dig()
        self.test_front_drum_stop()
        self.test_front_drum_dump()
        self.test_front_drum_stop()
        
        # Back drum tests
        self.get_logger().info('\n--- BACK DRUM TESTS ---')
        self.test_back_drum_dig()
        self.test_front_drum_stop()
        self.test_back_drum_dump()
        self.test_front_drum_stop()
        
        # Routine tests
        self.get_logger().info('\n--- ROUTINE TESTS ---')
        self.test_routine_auto_drive()
        self.test_routine_stop()
        self.test_routine_auto_dig()
        self.test_routine_stop()
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('All tests completed!')
        self.get_logger().info('=' * 60)

    def run_interactive(self):
        """Run interactive test mode"""
        self.get_logger().info('\n=== INTERACTIVE MODE ===')
        self.get_logger().info('Commands:')
        self.get_logger().info('  w - forward    s - backward    q - stop wheels')
        self.get_logger().info('  a - left       d - right       e - exit')
        self.get_logger().info('  1 - front arm up    2 - front arm down    0 - stop front arm')
        self.get_logger().info('  3 - back arm up     4 - back arm down')
        self.get_logger().info('  5 - front drum dig  6 - front drum dump   9 - stop front drum')
        self.get_logger().info('  7 - back drum dig   8 - back drum dump')
        self.get_logger().info('=' * 60)
        
        try:
            import termios
            import tty
            
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            
            try:
                tty.setraw(sys.stdin.fileno())
                while True:
                    ch = sys.stdin.read(1)
                    
                    if ch == 'e' or ch == '\x03':  # e or Ctrl+C
                        break
                    elif ch == 'w':
                        msg = Twist()
                        msg.linear.x = 0.5
                        self.wheel_pub.publish(msg)
                        print('\rForward   ', flush=True)
                    elif ch == 's':
                        msg = Twist()
                        msg.linear.x = -0.5
                        self.wheel_pub.publish(msg)
                        print('\rBackward  ', flush=True)
                    elif ch == 'a':
                        msg = Twist()
                        msg.angular.z = 1.0
                        self.wheel_pub.publish(msg)
                        print('\rLeft      ', flush=True)
                    elif ch == 'd':
                        msg = Twist()
                        msg.angular.z = -1.0
                        self.wheel_pub.publish(msg)
                        print('\rRight     ', flush=True)
                    elif ch == 'q':
                        msg = Twist()
                        self.wheel_pub.publish(msg)
                        print('\rStop      ', flush=True)
                    elif ch == '1':
                        msg = Float64()
                        msg.data = 1.0
                        self.front_arm_pub.publish(msg)
                        print('\rFront arm up  ', flush=True)
                    elif ch == '2':
                        msg = Float64()
                        msg.data = -1.0
                        self.front_arm_pub.publish(msg)
                        print('\rFront arm down', flush=True)
                    elif ch == '0':
                        msg = Float64()
                        msg.data = 0.0
                        self.front_arm_pub.publish(msg)
                        print('\rFront arm stop', flush=True)
                    elif ch == '3':
                        msg = Float64()
                        msg.data = 1.0
                        self.back_arm_pub.publish(msg)
                        print('\rBack arm up   ', flush=True)
                    elif ch == '4':
                        msg = Float64()
                        msg.data = -1.0
                        self.back_arm_pub.publish(msg)
                        print('\rBack arm down ', flush=True)
                    elif ch == '5':
                        msg = Float64()
                        msg.data = 1.0
                        self.front_drum_pub.publish(msg)
                        print('\rFront drum dig', flush=True)
                    elif ch == '6':
                        msg = Float64()
                        msg.data = -1.0
                        self.front_drum_pub.publish(msg)
                        print('\rFront drum dump', flush=True)
                    elif ch == '9':
                        msg = Float64()
                        msg.data = 0.0
                        self.front_drum_pub.publish(msg)
                        print('\rFront drum stop', flush=True)
                    elif ch == '7':
                        msg = Float64()
                        msg.data = 1.0
                        self.back_drum_pub.publish(msg)
                        print('\rBack drum dig  ', flush=True)
                    elif ch == '8':
                        msg = Float64()
                        msg.data = -1.0
                        self.back_drum_pub.publish(msg)
                        print('\rBack drum dump ', flush=True)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                print('\n')
                
        except ImportError:
            self.get_logger().error('Interactive mode requires termios (Linux/Mac only)')
            self.get_logger().info('Running automated tests instead...')
            self.run_all_tests()


def main(args=None):
    rclpy.init(args=args)
    tester = MotorControllerTester()
    
    # Determine test mode
    mode = 'comprehensive'
    if len(sys.argv) > 1:
        if sys.argv[1] == '--interactive' or sys.argv[1] == '-i':
            mode = 'interactive'
        elif sys.argv[1] == '--quick' or sys.argv[1] == '-q':
            mode = 'quick'
        elif sys.argv[1] == '--verify-only':
            mode = 'verify'
    
    # Setup signal handler for clean shutdown
    def signal_handler(sig, frame):
        print('\n\nShutdown requested...')
        tester.stop_motor_controller()
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start motor_controller node
        if not tester.start_motor_controller():
            print('Failed to start motor_controller, exiting...')
            return 1
        
        # Verify node is running
        if not tester.verify_node_running():
            print('Node verification failed, exiting...')
            tester.stop_motor_controller()
            return 1
        
        # Verify topics exist
        if not tester.verify_topics():
            print('Topic verification failed, exiting...')
            tester.stop_motor_controller()
            return 1
        
        # Wait for connections
        tester.wait_for_connections()
        
        # Run tests based on mode
        if mode == 'verify':
            print('\nVerification complete! Node is running with all topics.')
        elif mode == 'interactive':
            tester.run_interactive()
        elif mode == 'quick':
            tester.run_quick_test()
        else:
            tester.run_all_tests()
        
        # Clean shutdown
        tester.stop_motor_controller()
        tester.destroy_node()
        rclpy.shutdown()
        
        print('\nTest script completed successfully')
        return 0
        
    except Exception as e:
        print(f'\nTest failed with error: {e}')
        tester.stop_motor_controller()
        tester.destroy_node()
        rclpy.shutdown()
        return 1


if __name__ == '__main__':
    sys.exit(main())