import numpy as np
from flask import Flask, request, Response, jsonify, stream_with_context
from flask_cors import CORS
import matplotlib.pyplot as plt
import threading
import socket
import time

rover_position = np.array([0.0, 0.0])
rover_heading = 0.0
rover_velocity = 0.0

last_update_time = time.time()


OBSTACLE_CENTER = np.array([5.0, 5.0])
OBSTACLE_SIZE = 1.0

last_command_status = None
command_count = 0


def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return "ip_" + ip.replace(".", "_")
    except:
        return "ip_sim"

ROVER_NAME = get_ip_address()

def generate_cube_pointcloud():

    half = OBSTACLE_SIZE / 2

    xs = np.linspace(-half, half, 20)
    ys = np.linspace(-half, half, 20)
    zs = np.linspace(0, 1, 20)

    pts = []

    for x in xs:
        for y in ys:
            for z in zs:
                pts.append([
                    OBSTACLE_CENTER[0] + x,
                    OBSTACLE_CENTER[1] + y,
                    z
                ])

    return np.array(pts, dtype=np.float32)

POINT_CLOUD = generate_cube_pointcloud()

# -----------------------------
# Motion simulation
# -----------------------------

def process_request(data):

    global rover_position
    global rover_heading
    global rover_velocity
    global last_update_time
    global last_command_status
    global command_count

    if data is None:
        raise ValueError("Invalid request")

    now = time.time()
    dt = now - last_update_time
    last_update_time = now

    if "wheel_action" in data:

        lin = float(data["wheel_action"].get("linear_x", 0))
        ang = float(data["wheel_action"].get("angular_z", 0))

        # smooth acceleration (real rover inertia)
        rover_velocity += (lin - rover_velocity) * 0.3

        rover_heading += ang * dt

        rover_position[0] += np.cos(rover_heading) * rover_velocity * dt
        rover_position[1] += np.sin(rover_heading) * rover_velocity * dt

    command_count += 1
    last_command_status = f"command_ok_{command_count}"

# -----------------------------
# Depth camera simulation
# -----------------------------

def generate_depth():

    width = 640
    height = 480

    depth = np.full((height, width), 65535, dtype=np.uint16)

    fx = 525
    fy = 525
    cx = width / 2
    cy = height / 2

    for p in POINT_CLOUD:

        dx = p[0] - rover_position[0]
        dy = p[1] - rover_position[1]

        # transform world -> rover frame
        x = np.cos(-rover_heading) * dx - np.sin(-rover_heading) * dy
        y = np.sin(-rover_heading) * dx + np.cos(-rover_heading) * dy
        z = p[2]

        if x <= 0:
            continue

        u = int(fx * (y/x) + cx)
        v = int(fy * (z/x) + cy)

        if 0 <= u < width and 0 <= v < height:

            d = int(x * 1000)

            if d < depth[v, u]:
                depth[v, u] = d

    return depth

def visualizer():

    plt.ion()
    fig, ax = plt.subplots()

    while True:

        ax.clear()

        ax.scatter(
            rover_position[0],
            rover_position[1],
            s=200,
            label="Rover"
        )

        ax.scatter(
            POINT_CLOUD[:,0],
            POINT_CLOUD[:,1],
            s=1,
            alpha=0.2,
            label="Obstacle"
        )

        ax.set_xlim(-10,10)
        ax.set_ylim(-10,10)

        ax.set_title("EZRASSOR Simulation")

        ax.legend()

        plt.pause(0.05)


app = Flask(__name__)
CORS(app)

@app.route("/", methods=["OPTIONS"])
def handle_options():
    return {"status":200}

@app.route("/", methods=["GET"])
def default_get():
    return {"status":200}

@app.route("/", methods=["POST"])
def handle_request():

    try:

        process_request(request.get_json())

        return {"status":200}

    except Exception as e:

        print("VerificationError:", e)

        return {"status":400}

@app.route(f"/{ROVER_NAME}/command_status", methods=["GET"])
def get_command_status():

    if last_command_status is None:
        return jsonify({'error':'No status received yet'}),400

    return jsonify({'status':last_command_status}),200

@app.route("/depth")
def depth():

    depth = generate_depth()

    return Response(
        depth.tobytes(),
        mimetype="application/octet-stream",
        headers={
            "Width":"640",
            "Height":"480",
            "Encoding":"16UC1"
        }
    )

@app.route('/video_feed')
def video_feed():

    def blank_stream():
        while True:
            yield b''

    return Response(
        stream_with_context(blank_stream()),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route('/is_detection')
def is_detection():
    return Response("none", mimetype="text/plain")

@app.after_request
def apply_cors_headers(response):

    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"

    return response

if __name__ == "__main__":

    viz_thread = threading.Thread(target=visualizer, daemon=True)
    viz_thread.start()

    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)