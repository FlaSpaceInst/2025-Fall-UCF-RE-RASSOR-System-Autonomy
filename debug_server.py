#!/usr/bin/env python3
"""
Simple HTTP POST test that matches your Flask server
"""

import requests
import sys

def test_connection(server_ip, port):
    base_url = f"http://{server_ip}:{port}"

    print("=" * 60)
    print(f"Testing HTTP connection to {base_url}")
    print("=" * 60)

    # 1. Test root endpoint
    print("\n1. Testing GET / ...")
    try:
        response = requests.get(f"{base_url}/", timeout=2)
        print(f"Status: {response.status_code}")
        print(f"Response: {response.json()}")
    except Exception as e:
        print(f"Error: {e}")
        return False

    # 2. Test wheel command
    print("\n2. Testing wheel command via POST / ...")
    wheel_payload = {
        "wheel_action": {
            "linear_x": 0.0,
            "angular_z": 0.0
        }
    }

    try:
        response = requests.post(
            f"{base_url}/",
            json=wheel_payload,
            timeout=2
        )
        print(f"Status: {response.status_code}")
        print(f"Response: {response.json()}")
    except Exception as e:
        print(f"Error: {e}")
        return False

    print("\nAll tests completed successfully.")
    return True


if __name__ == "__main__":
    port = 5000
    server_ip = "192.168.1.11"

    if len(sys.argv) > 1:
        server_ip = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])

    success = test_connection(server_ip, port)

    if not success:
        sys.exit(1)