# depth_server.py  --  Orbbec Astra Pro 3D  +  EZRASSOR rover controller

import struct
import threading
import time
import logging
import socket
import multiprocessing
from collections import deque
from typing import Optional

# ---------------------------------------------------------------------------
# Port helper
# ---------------------------------------------------------------------------
def _find_free_port(preferred, host="0.0.0.0"):
    # type: (int, str) -> int
    """Try preferred port; if taken let the OS pick the next free one."""
    import socket as _s
    sock = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
    sock.setsockopt(_s.SOL_SOCKET, _s.SO_REUSEADDR, 1)
    try:
        sock.bind((host, preferred))
        sock.close()
        return preferred
    except OSError:
        sock2 = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
        sock2.bind((host, 0))
        port = sock2.getsockname()[1]
        sock2.close()
        return port


OPENNI_LIB_PATH = (
    r"C:\Users\nmist\Downloads\Orbbec_OpenNI_v2.3.0.86-beta6_windows_release"
    r"\OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows\OpenNI2\sdk\libs"
)

HTTP_HOST = "0.0.0.0"
WS_HOST   = "0.0.0.0"

_HTTP_PORT_PREFERRED = 8000
_WS_PORT_PREFERRED   = 8765

HEADER_MAGIC = 0x52474244
HEADER_SIZE  = 32
TRAIL_LENGTH = 200

WS_TARGET_FPS     = 10
WS_FRAME_INTERVAL = 1.0 / WS_TARGET_FPS

COLOR_CAMERA_INDEX = 1

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)


def _get_rover_name():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return "ip_" + ip.replace(".", "_")
    except Exception:
        return "ip_sim"


def _flask_process_fn(rover_pos, rover_heading, cmd_count, cmd_status,
                      rover_name, ws_port, http_host, http_port,
                      header_size, ws_target_fps, width, height):
    # type: (...) -> None
    import struct as _struct
    import math as _math
    import flask as _flask
    from flask import request, Response, jsonify, stream_with_context
    from flask_cors import CORS

    app = _flask.Flask(__name__)
    CORS(app)

    def _process_command(data):
        if data is None:
            raise ValueError("Invalid request body")
        if "wheel_action" in data:
            lin = float(data["wheel_action"].get("linear_x", 0))
            ang = float(data["wheel_action"].get("angular_z", 0))
            with rover_heading.get_lock():
                rover_heading.value += ang * 0.1
                hdg = rover_heading.value
            with rover_pos.get_lock():
                rover_pos[0] += _math.cos(hdg) * lin * 0.1
                rover_pos[1] += _math.sin(hdg) * lin * 0.1
        if "target_coordinate" in data:
            tx = float(data["target_coordinate"]["x"])
            ty = float(data["target_coordinate"]["y"])
            with rover_pos.get_lock():
                rover_pos[0] = tx
                rover_pos[1] = ty
        with cmd_count.get_lock():
            cmd_count.value += 1
            n = cmd_count.value
        status_bytes = f"command_ok_{n}".encode().ljust(64)[:64]
        for i in range(64):
            cmd_status[i] = status_bytes[i:i+1]

    @app.route("/", methods=["OPTIONS"])
    def handle_options():
        return {"status": 200}

    @app.route("/", methods=["GET"])
    def default_get():
        return {"status": 200}

    @app.route("/", methods=["POST"])
    def handle_request():
        try:
            _process_command(request.get_json())
            return {"status": 200}
        except Exception as err:
            print("CommandError:", err)
            return {"status": 400}

    @app.route(f"/{rover_name}/command_status", methods=["GET"])
    def get_command_status():
        raw = b"".join(cmd_status).rstrip(b"\x00 ")
        if not raw:
            return jsonify({"error": "No status received yet"}), 400
        return jsonify({"status": raw.decode()}), 200

    @app.route("/depth")
    def depth_endpoint():
        try:
            import websockets.sync.client as _wsc
            with _wsc.connect(f"ws://127.0.0.1:{ws_port}", open_timeout=2) as _ws:
                blob = _ws.recv()
            _, w, h, depth_b, _, _, _ = _struct.unpack_from("<IIIIIII", blob)
            raw = blob[header_size: header_size + depth_b]
            return Response(raw, mimetype="application/octet-stream",
                            headers={"Width": str(w), "Height": str(h),
                                     "Encoding": "16UC1"})
        except Exception as ex:
            return Response(f"Error: {ex}", status=503)

    @app.route("/status")
    def status_endpoint():
        return {"width": width, "height": height, "ws_port": ws_port,
                "fps_target": ws_target_fps,
                "rover_x": rover_pos[0], "rover_y": rover_pos[1],
                "rover_yaw": rover_heading.value}

    @app.route("/video_feed")
    def video_feed():
        def blank_stream():
            while True:
                yield b""
        return Response(stream_with_context(blank_stream()),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/is_detection")
    def is_detection():
        return Response("none", mimetype="text/plain")

    @app.after_request
    def apply_cors(response):
        response.headers["Access-Control-Allow-Origin"]  = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"
        return response

    app.run(host=http_host, port=http_port, debug=False, threaded=True)



def run_server():
    # type: () -> None
    import numpy as np
    import cv2
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.gridspec as gridspec
    from openni import openni2
    from openni import _openni2 as c_api
    import websockets.sync.server as ws_sync

    # ── Resolve ports ─────────────────────────────────────────────────────
    http_port = _find_free_port(_HTTP_PORT_PREFERRED)
    ws_port   = _find_free_port(_WS_PORT_PREFERRED)

    rover_name = _get_rover_name()

    # ── Shared state ──────────────────────────────────────────────────────
    rover_pos     = multiprocessing.Array("d", [0.0, 0.0])
    rover_heading = multiprocessing.Value("d", 0.0)
    cmd_count     = multiprocessing.Value("i", 0)
    cmd_status    = multiprocessing.Array("c", b" " * 64)
    depth_stats   = multiprocessing.Array("d", [0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    ws_client_count = multiprocessing.Value("i", 0)

    frame_lock   = threading.Lock()
    latest_blob  = [None]   # type: list  -- list so closure can rebind
    new_frame    = threading.Event()

    # ── OpenNI2 ───────────────────────────────────────────────────────────
    openni2.initialize(OPENNI_LIB_PATH)
    dev = openni2.Device.open_any()
    log.info("OpenNI device: %s", dev.get_device_info())

    depth_stream = dev.create_depth_stream()
    depth_stream.set_video_mode(c_api.OniVideoMode(
        pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
        resolutionX=640, resolutionY=480, fps=30,
    ))
    depth_stream.start()

    log.info("Draining warm-up frames (max 3 s)...")
    deadline = time.time() + 3.0
    drained  = 0
    while time.time() < deadline:
        if openni2.wait_for_any_stream([depth_stream], timeout=100):
            depth_stream.read_frame()
            drained += 1
            if drained >= 30:
                break
    log.info("Depth stream ready (drained %d frames)", drained)

    width  = depth_stream.get_video_mode().resolutionX
    height = depth_stream.get_video_mode().resolutionY
    log.info("Depth resolution: %d x %d", width, height)

    # ── OpenCV color ──────────────────────────────────────────────────────
    color_cap = None  # type: Optional[cv2.VideoCapture]
    try:
        cap = cv2.VideoCapture(COLOR_CAMERA_INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, 30)
            ok, f = cap.read()
            if ok and f is not None:
                color_cap = cap
                log.info("Color capture ok (index %d): %dx%d",
                         COLOR_CAMERA_INDEX, f.shape[1], f.shape[0])
            else:
                cap.release()
                log.warning("Color index %d: no frame", COLOR_CAMERA_INDEX)
        else:
            log.warning("Color index %d: failed to open", COLOR_CAMERA_INDEX)
    except Exception as e:
        log.warning("Color init error: %s", e)

    def _read_color():
        # type: () -> bytes
        n = width * height * 3
        if color_cap is None:
            return bytes(n)
        ok, bgr = color_cap.read()
        if not ok or bgr is None:
            return bytes(n)
        if bgr.shape[1] != width or bgr.shape[0] != height:
            bgr = cv2.resize(bgr, (width, height))
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()

    # ── Capture thread ────────────────────────────────────────────────────
    depth_expected = width * height * 2

    def _capture_fn():
        log_counter = 0
        log.info("Capture thread entering read loop")
        while True:
            if not openni2.wait_for_any_stream([depth_stream], timeout=200):
                log.warning("wait_for_any_stream timeout")
                continue
            try:
                d_frame   = depth_stream.read_frame()
                depth_arr = np.array(d_frame.get_buffer_as_uint16(), dtype=np.uint16)
                del d_frame
            except Exception as e:
                log.warning("Depth read error: %s", e)
                continue
            if depth_arr.nbytes != depth_expected:
                continue
            nz = int(np.count_nonzero(depth_arr))
            if nz == 0:
                log.warning("Depth ALL ZEROS")
                continue
            v = depth_arr[depth_arr > 0]
            depth_stats[0] = float(nz)
            depth_stats[1] = float(len(depth_arr))
            depth_stats[2] = float(v.min())
            depth_stats[3] = float(v.max())
            depth_stats[4] = float(v.mean())
            depth_stats[5] = float(log_counter)
            if log_counter % 150 == 0:
                log.info("Depth OK | nz=%d/%d | min=%dmm max=%dmm mean=%.0fmm",
                         nz, len(depth_arr), int(v.min()), int(v.max()), float(v.mean()))
            log_counter += 1
            depth_raw = depth_arr.tobytes()
            color_raw = _read_color()
            ts_ms = int(time.time() * 1000) & 0xFFFFFFFF
            header = struct.pack("<IIIIIII",
                HEADER_MAGIC, width, height,
                len(depth_raw), len(color_raw), ts_ms, 0,
            ) + b"\x00\x00\x00\x00"
            with frame_lock:
                latest_blob[0] = header + depth_raw + color_raw
            new_frame.set()

    threading.Thread(target=_capture_fn, daemon=True, name="capture").start()
    log.info("Capture thread started (30 fps native)")

    # ── WebSocket server ──────────────────────────────────────────────────
    ws_clients      = set()
    ws_clients_lock = threading.Lock()

    def _ws_handler(ws):
        addr = ws.remote_address
        log.info("WS connected: %s", addr)
        with ws_clients_lock:
            ws_clients.add(ws)
            ws_client_count.value = len(ws_clients)
        last_sent      = None
        last_send_time = 0.0
        try:
            while True:
                now    = time.monotonic()
                wait_s = max(0.0, WS_FRAME_INTERVAL - (now - last_send_time))
                if not new_frame.wait(timeout=wait_s + 0.5):
                    continue
                new_frame.clear()
                with frame_lock:
                    blob = latest_blob[0]
                if blob is None or blob is last_sent:
                    continue
                if (time.monotonic() - last_send_time) < WS_FRAME_INTERVAL * 0.9:
                    continue
                ws.send(blob)
                last_sent      = blob
                last_send_time = time.monotonic()
        except Exception as e:
            log.debug("WS %s disconnected: %s", addr, e)
        finally:
            with ws_clients_lock:
                ws_clients.discard(ws)
                ws_client_count.value = len(ws_clients)
            log.info("WS disconnected: %s", addr)

    def _run_ws():
        log.info("WebSocket server on ws://%s:%d", WS_HOST, ws_port)
        try:
            with ws_sync.serve(_ws_handler, WS_HOST, ws_port) as server:
                server.serve_forever()
        except OSError as e:
            log.error("WS bind failed on port %d: %s", ws_port, e)
            log.error("Run:  netstat -ano | findstr :%d  then  taskkill /PID <PID> /F",
                      _WS_PORT_PREFERRED)
            raise

    threading.Thread(target=_run_ws, daemon=True, name="ws").start()

    # ── Flask process ─────────────────────────────────────────────────────
    flask_proc = multiprocessing.Process(
        target=_flask_process_fn, daemon=True, name="flask",
        args=(rover_pos, rover_heading, cmd_count, cmd_status,
              rover_name, ws_port, HTTP_HOST, http_port,
              HEADER_SIZE, WS_TARGET_FPS, width, height))
    flask_proc.start()
    log.info("Flask process started on http://%s:%d  (rover: %s)",
             HTTP_HOST, http_port, rover_name)

    # ── Startup banner ────────────────────────────────────────────────────
    log.info("")
    log.info("=" * 55)
    log.info("  EZRASSOR Depth Server")
    log.info("  HTTP : http://0.0.0.0:%d  (preferred %d)",
             http_port, _HTTP_PORT_PREFERRED)
    log.info("  WS   : ws://0.0.0.0:%d  (preferred %d)",
             ws_port, _WS_PORT_PREFERRED)
    log.info("  Rover: %s", rover_name)
    if http_port != _HTTP_PORT_PREFERRED or ws_port != _WS_PORT_PREFERRED:
        log.warning("Preferred ports were taken -- ports reassigned above.")
        log.warning("Update depth_costmap_node ws_port param to: %d", ws_port)
        log.warning("To free original ports on Windows:")
        log.warning("  netstat -ano | findstr :%d", _WS_PORT_PREFERRED)
        log.warning("  taskkill /PID <PID> /F")
    log.info("=" * 55)
    log.info("")

    # ── Matplotlib visualizer  (must run on main thread on Windows) ───────
    DARK_BG    = "#1a1a2e"
    PANEL_BG   = "#16213e"
    ACCENT     = "#e94560"
    GREEN      = "#0f9b58"
    BLUE       = "#0f3460"
    TEXT_COLOR = "#e0e0e0"
    GRID_COLOR = "#2a2a4a"

    plt.ion()
    fig = plt.figure(figsize=(12, 8), facecolor=DARK_BG)
    fig.canvas.manager.set_window_title("EZRASSOR Depth Server Monitor")

    gs = gridspec.GridSpec(2, 2, figure=fig,
                           hspace=0.45, wspace=0.35,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)
    ax_map    = fig.add_subplot(gs[0, 0])
    ax_depth  = fig.add_subplot(gs[0, 1])
    ax_cmds   = fig.add_subplot(gs[1, 0])
    ax_status = fig.add_subplot(gs[1, 1])

    for ax in [ax_map, ax_depth, ax_cmds, ax_status]:
        ax.set_facecolor(PANEL_BG)
        ax.tick_params(colors=TEXT_COLOR, labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID_COLOR)

    fig.suptitle("EZRASSOR  |  Depth Server Monitor",
                 color=TEXT_COLOR, fontsize=13, fontweight="bold", y=0.97)

    # Map
    ax_map.set_title("Rover Map", color=TEXT_COLOR, fontsize=9, pad=4)
    ax_map.set_xlabel("X (right, m)", color=TEXT_COLOR, fontsize=8)
    ax_map.set_ylabel("Y (forward, m)", color=TEXT_COLOR, fontsize=8)
    ax_map.grid(True, color=GRID_COLOR, linewidth=0.5)
    ax_map.set_aspect("equal")
    ax_map.axhline(0, color=GRID_COLOR, linewidth=0.8)
    ax_map.axvline(0, color=GRID_COLOR, linewidth=0.8)
    trail_x, trail_y = deque(maxlen=TRAIL_LENGTH), deque(maxlen=TRAIL_LENGTH)
    trail_line, = ax_map.plot([], [], "-", color=BLUE, linewidth=1.5, alpha=0.7, label="Trail")
    rover_dot,  = ax_map.plot([], [], "o", color=ACCENT, markersize=10, zorder=5, label="Rover")
    heading_arrow = ax_map.quiver(0, 0, 0, 1, color=GREEN, scale=5, width=0.012, zorder=6)
    ax_map.legend(loc="upper right", fontsize=7,
                  facecolor=PANEL_BG, labelcolor=TEXT_COLOR, edgecolor=GRID_COLOR)

    # Depth bars
    ax_depth.set_title("Depth Stats", color=TEXT_COLOR, fontsize=9, pad=4)
    ax_depth.set_ylabel("Value", color=TEXT_COLOR, fontsize=8)
    ax_depth.grid(axis="y", color=GRID_COLOR, linewidth=0.5)
    depth_labels = ["Fill %", "Min m", "Mean m", "Max m"]
    depth_bars = ax_depth.bar(depth_labels, [0]*4,
                              color=[GREEN, BLUE, ACCENT, "#f5a623"],
                              edgecolor=DARK_BG, linewidth=0.5)
    ax_depth.set_ylim(0, 12)
    depth_val_texts = [
        ax_depth.text(i, 0.1, "", ha="center", va="bottom",
                      color=TEXT_COLOR, fontsize=7, fontweight="bold")
        for i in range(4)
    ]

    # Command rate
    ax_cmds.set_title("Commands (last 60 s)", color=TEXT_COLOR, fontsize=9, pad=4)
    ax_cmds.set_xlabel("Time (s ago)", color=TEXT_COLOR, fontsize=8)
    ax_cmds.set_ylabel("Cmds/s", color=TEXT_COLOR, fontsize=8)
    ax_cmds.set_xlim(0, 60)
    ax_cmds.set_ylim(0, 10)
    ax_cmds.grid(axis="y", color=GRID_COLOR, linewidth=0.5)
    cmd_bins = np.zeros(60)
    cmd_bar_container = ax_cmds.bar(np.arange(60), cmd_bins,
                                    color=ACCENT, edgecolor=DARK_BG,
                                    linewidth=0.3, width=0.9)

    # Status text
    ax_status.set_title("Live Status", color=TEXT_COLOR, fontsize=9, pad=4)
    ax_status.axis("off")
    status_text = ax_status.text(
        0.05, 0.95, "", transform=ax_status.transAxes,
        color=TEXT_COLOR, fontsize=9, fontfamily="monospace",
        va="top", ha="left",
        bbox=dict(boxstyle="round,pad=0.5", facecolor=BLUE,
                  edgecolor=ACCENT, alpha=0.8))

    import math
    cmd_times = deque(maxlen=60)
    last_cmd  = -1
    last_draw = time.monotonic()

    log.info("Visualizer running (close window or Ctrl+C to stop)")

    while plt.fignum_exists(fig.number):
        now_t = time.monotonic()
        if now_t - last_draw < 0.1:
            plt.pause(0.02)
            continue
        last_draw = now_t

        rx   = rover_pos[0]
        ry   = rover_pos[1]
        hdg  = rover_heading.value
        ncmd = cmd_count.value
        nws  = ws_client_count.value
        nz   = depth_stats[0]
        tot  = max(depth_stats[1], 1.0)
        dmin = depth_stats[2]
        dmax = depth_stats[3]
        dmean= depth_stats[4]
        fnum = int(depth_stats[5])
        raw_status = b"".join(cmd_status).rstrip(b"\x00 ").decode(errors="replace")

        # Trail + rover dot
        trail_x.append(rx)
        trail_y.append(ry)
        trail_line.set_data(list(trail_x), list(trail_y))
        rover_dot.set_data([rx], [ry])

        span = 5.0
        ax_map.set_xlim(rx - span, rx + span)
        ax_map.set_ylim(ry - span, ry + span)

        dx = math.sin(hdg) * 0.8
        dy = math.cos(hdg) * 0.8
        heading_arrow.set_offsets([[rx, ry]])
        heading_arrow.set_UVC([dx], [dy])

        # Depth bars
        fill_pct   = (nz / tot) * 100.0
        dvals      = [fill_pct, dmin/1000.0, dmean/1000.0, dmax/1000.0]
        depth_ylim = max(12.0, dmax/1000.0 * 1.2)
        ax_depth.set_ylim(0, depth_ylim)
        for bar, val, txt, idx in zip(depth_bars, dvals, depth_val_texts, range(4)):
            bar.set_height(val)
            label = f"{val:.1f}%" if idx == 0 else f"{val:.2f}m"
            txt.set_text(label)
            txt.set_y(val + depth_ylim * 0.01)

        # Command histogram
        if ncmd != last_cmd:
            cmd_times.append(now_t)
            last_cmd = ncmd
        cmd_bins[:] = 0
        for ct in cmd_times:
            age = int(now_t - ct)
            if 0 <= age < 60:
                cmd_bins[age] += 1
        ax_cmds.set_ylim(0, max(cmd_bins.max() * 1.3, 3))
        for rect, h in zip(cmd_bar_container, cmd_bins):
            rect.set_height(h)
            rect.set_facecolor(ACCENT if h > 0 else GRID_COLOR)

        # Status text
        status_text.set_text("\n".join([
            f"  Rover position",
            f"    X (right)  : {rx:+.3f} m",
            f"    Y (forward): {ry:+.3f} m",
            f"    Heading    : {math.degrees(hdg):+.1f} deg",
            f"",
            f"  Commands",
            f"    Total      : {ncmd}",
            f"    Last status: {raw_status or 'none'}",
            f"",
            f"  Depth camera",
            f"    Fill       : {fill_pct:.1f} %",
            f"    Range      : {dmin/1000:.2f} - {dmax/1000:.2f} m",
            f"    Mean       : {dmean/1000:.2f} m",
            f"    Frame #    : {fnum}",
            f"",
            f"  WebSocket",
            f"    Clients    : {nws}",
            f"    Port       : {ws_port}",
            f"",
            f"  HTTP",
            f"    Port       : {http_port}",
            f"    Rover ID   : {rover_name}",
        ]))

        fig.canvas.draw_idle()
        plt.pause(0.02)

if __name__ == "__main__":
    multiprocessing.freeze_support()
    run_server()