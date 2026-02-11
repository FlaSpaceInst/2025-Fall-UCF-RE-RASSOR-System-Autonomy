#!/usr/bin/env python3
"""
Simple HTTP POST test to verify connectivity
Run this inside Docker to test if you can reach the HTTP server
"""

import requests
import json
import sys

def test_connection(server_ip, port):
    """Test HTTP connection to server"""
    base_url = f"http://{server_ip}:{port}"
    
    print("=" * 60)
    print(f"Testing HTTP connection to {base_url}")
    print("=" * 60)
    
    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/health", timeout=2)
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        print("   ✓ Health check passed")
    except requests.exceptions.ConnectionError:
        print(f"   ✗ Connection refused - server not reachable at {base_url}")
        print(f"   Make sure HTTP server is running on host at port {port}")
        return False
    except Exception as e:
        print(f"   ✗ Error: {e}")
        return False
    
    # Test 2: Send a wheel command
    print("\n2. Testing wheel command endpoint...")
    try:
        payload = {
            "linear_x": 0.5,
            "angular_z": 0.0,
            "timestamp": 1234567890.123
        }
        response = requests.post(
            f"{base_url}/wheel_command",
            json=payload,
            timeout=2
        )
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        print("   ✓ Wheel command sent successfully")
    except Exception as e:
        print(f"   ✗ Error: {e}")
        return False
    
    # Test 3: Send an arm command
    print("\n3. Testing arm command endpoint...")
    try:
        payload = {
            "arm": "front",
            "action": 1.0,
            "timestamp": 1234567890.123
        }
        response = requests.post(
            f"{base_url}/arm_command",
            json=payload,
            timeout=2
        )
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        print("   ✓ Arm command sent successfully")
    except Exception as e:
        print(f"   ✗ Error: {e}")
        return False
    
    print("\n" + "=" * 60)
    print("✓ All HTTP tests passed!")
    print("=" * 60)
    return True

def find_working_address(port):
    """Try multiple addresses to find working one"""
    addresses = [
        "host.docker.internal",
        "172.17.0.1",  # Default Docker bridge
        "10.32.24.142", # Your host IP from server output
        "localhost",
        "127.0.0.1"
    ]
    
    print("Searching for accessible server address...")
    for addr in addresses:
        try:
            response = requests.get(f"http://{addr}:{port}/health", timeout=1)
            if response.status_code == 200:
                print(f"✓ Found server at: {addr}")
                return addr
        except:
            pass
    
    return None

if __name__ == "__main__":
    port = 5001
    server_ip = "host.docker.internal"
    
    if len(sys.argv) > 1:
        server_ip = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    
    # First try to find a working address
    working_addr = find_working_address(port)
    
    if working_addr:
        print(f"\nUsing address: {working_addr}\n")
        success = test_connection(working_addr, port)
        
        if success:
            print(f"\nTo use this address in motor_controller, run:")
            print(f"   ros2 run re_rassor_motor_controller motor_controller --ros-args \\")
            print(f"       -p server_ip:={working_addr} -p server_port:={port}")
    else:
        print("\n✗ Could not find accessible server")
        print(f"   Tried: {', '.join(['host.docker.internal', '172.17.0.1', '10.32.24.142', 'localhost'])}")
        print("\nMake sure HTTP server is running on host:")
        print("   ./http_server.py")
        sys.exit(1)