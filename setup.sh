#!/usr/bin/env bash
# =============================================================================
# RE-RASSOR Rover Setup Script
# Target: Raspberry Pi — Ubuntu 24.04 LTS (Noble) — ROS2 Jazzy
#
# Run once on a fresh Pi:
#   sudo bash setup.sh
#
# Idempotent — safe to re-run if something fails mid-way.
# =============================================================================
set -euo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
ROS_DISTRO="jazzy"
ROVER_USER="${SUDO_USER:-ubuntu}"                        # the non-root user
ROVER_HOME="/home/${ROVER_USER}"
WS_DIR="${ROVER_HOME}/ros2_ws"
REPO_DIR="${WS_DIR}/2025-Fall-UCF-RE-RASSOR-System-Autonomy"
REPO_URL="https://github.com/UCF-Team22-Senior-Design/2025-Fall-UCF-RE-RASSOR-System-Autonomy.git"

# Arduino USB port symlink names — these match the KERNELS ids in 01-arduino.rules.
# If the Arduinos enumerate on different physical USB ports after install,
# edit /etc/udev/rules.d/01-arduino.rules and change KERNELS=="1-2" / KERNELS=="1-1.1"
# to match.  Run:  udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
WHEEL_USB_KERNEL="1-2"
DRUM_USB_KERNEL="1-1.1"

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

step()  { echo -e "\n${CYAN}${BOLD}▶ $*${NC}"; }
ok()    { echo -e "  ${GREEN}✔ $*${NC}"; }
warn()  { echo -e "  ${YELLOW}⚠ $*${NC}"; }
die()   { echo -e "\n${RED}✖ $*${NC}" >&2; exit 1; }

# ── Guards ────────────────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || die "Run with sudo:  sudo bash setup.sh"

# Verify Ubuntu 24.04
if ! grep -q 'Ubuntu 24' /etc/os-release 2>/dev/null; then
    warn "This script targets Ubuntu 24.04. Detected:"
    grep PRETTY_NAME /etc/os-release || true
    read -r -p "  Continue anyway? [y/N] " yn
    [[ "$yn" =~ ^[Yy]$ ]] || exit 1
fi

echo -e "\n${BOLD}RE-RASSOR Rover Setup${NC}"
echo    "  User      : ${ROVER_USER}"
echo    "  Workspace : ${WS_DIR}"
echo    "  ROS       : ${ROS_DISTRO}"

# =============================================================================
# 1. System update
# =============================================================================
step "System update"
apt-get update -qq
apt-get upgrade -y -qq
ok "System up to date"

# =============================================================================
# 2. Core utilities
# =============================================================================
step "Core utilities"
apt-get install -y -qq \
    curl wget git build-essential cmake \
    python3-pip python3-venv python3-dev \
    software-properties-common \
    udev \
    can-utils net-tools
ok "Core utilities installed"

# =============================================================================
# 3. ROS2 Jazzy
# =============================================================================
step "ROS2 Jazzy"

if dpkg -l ros-jazzy-ros-base &>/dev/null; then
    ok "ROS2 Jazzy already installed — skipping"
else
    # Add ROS2 apt repo
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list
    apt-get update -qq
    apt-get install -y -qq ros-jazzy-ros-base
    ok "ROS2 Jazzy base installed"
fi

# =============================================================================
# 4. ROS2 nav + SLAM packages
# =============================================================================
step "ROS2 navigation and SLAM packages"
apt-get install -y -qq \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-launch \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-msgs
ok "Nav2 + RTAB-Map installed"

# =============================================================================
# 5. ROS2 perception / TF packages
# =============================================================================
step "ROS2 perception and TF packages"
apt-get install -y -qq \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-image-publisher \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-action \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros2action \
    ros-${ROS_DISTRO}-ros2topic \
    ros-${ROS_DISTRO}-ros2service \
    ros-${ROS_DISTRO}-ros2run
ok "Perception and TF packages installed"

# =============================================================================
# 6. Build tools
# =============================================================================
step "Colcon + rosdep"
apt-get install -y -qq \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-catkin-pkg \
    python3-ament-package

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi
sudo -u "${ROVER_USER}" rosdep update
ok "Colcon and rosdep ready"

# =============================================================================
# 7. C++ build dependencies (Astra camera + motor controller)
# =============================================================================
step "C++ library dependencies"

# Packages from README: libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
# NOTE: do NOT install libuvc-dev from apt — it is too old.
#       libuvc is built from source in step 7b below (per ros2_astra_camera README).
apt-get install -y -qq \
    libgflags-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libopencv-dev \
    libusb-1.0-0-dev \
    libnlohmann-json3-dev \
    libwebsockets-dev \
    pkg-config
ok "C++ apt libraries installed"

# ── 7b. libuvc from source (required by ros2_astra_camera README) ─────────────
step "libuvc — build from source"

LIBUVC_DIR="/tmp/libuvc"
if ldconfig -p | grep -q libuvc; then
    ok "libuvc already installed — skipping"
else
    rm -rf "${LIBUVC_DIR}"
    git clone --depth 1 https://github.com/libuvc/libuvc.git "${LIBUVC_DIR}"
    cmake -S "${LIBUVC_DIR}" -B "${LIBUVC_DIR}/build" -DCMAKE_BUILD_TYPE=Release
    make -C "${LIBUVC_DIR}/build" -j"$(nproc)"
    make -C "${LIBUVC_DIR}/build" install
    ldconfig
    ok "libuvc built and installed"
fi

# =============================================================================
# 8. Python dependencies (controller server)
# =============================================================================
step "Python dependencies"
# Use pip with --break-system-packages for Ubuntu 24 (PEP 668)
pip3 install --break-system-packages \
    flask \
    flask-cors \
    flask-socketio \
    python-socketio \
    eventlet \
    pyserial
ok "Python packages installed"

# =============================================================================
# 9. User groups (serial + USB + camera access)
# =============================================================================
step "User group membership"
usermod -aG dialout "${ROVER_USER}"   # serial ports (/dev/ttyUSB*, /dev/ttyACM*)
usermod -aG video   "${ROVER_USER}"   # camera
usermod -aG plugdev "${ROVER_USER}"   # USB devices
ok "${ROVER_USER} added to dialout, video, plugdev"

# =============================================================================
# 10. udev rules — Arduino serial symlinks
# =============================================================================
step "udev rules — Arduino"

cat > /etc/udev/rules.d/01-arduino.rules << EOF
# RE-RASSOR Arduino serial port symlinks
# KERNELS match the physical USB port position on the Pi.
# If wheels/drums don't appear after reboot, run:
#   udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
# and update the values below to match your port layout.
SUBSYSTEM=="tty", KERNELS=="${WHEEL_USB_KERNEL}", SYMLINK+="arduino_wheel", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNELS=="${DRUM_USB_KERNEL}",  SYMLINK+="arduino_drum",  MODE="0666", GROUP="dialout"
EOF

ok "Arduino udev rules written → /etc/udev/rules.d/01-arduino.rules"
warn "Wheel Arduino must be plugged into USB port kernel ${WHEEL_USB_KERNEL}"
warn "Drum  Arduino must be plugged into USB port kernel ${DRUM_USB_KERNEL}"
warn "Run 'udevadm info -a -n /dev/ttyUSB0 | grep KERNELS' to verify port IDs"

# Astra udev rules are installed in step 12b (after repo clone),
# exactly as ros2_astra_camera README specifies:
#   cd .../astra_camera/scripts && sudo bash install.sh

# =============================================================================
# 12. Clone / update workspace
# =============================================================================
step "Workspace — ${WS_DIR}"

mkdir -p "${WS_DIR}"
chown "${ROVER_USER}:${ROVER_USER}" "${WS_DIR}"

if [[ -d "${REPO_DIR}/.git" ]]; then
    ok "Repo already cloned — pulling latest"
    sudo -u "${ROVER_USER}" git -C "${REPO_DIR}" pull --ff-only || \
        warn "git pull failed (local changes?). Skipping pull."
else
    sudo -u "${ROVER_USER}" git clone "${REPO_URL}" "${REPO_DIR}"
    ok "Repo cloned → ${REPO_DIR}"
fi

# ── 12b. Astra udev rules — run install.sh as specified in README ─────────────
step "udev rules — Orbbec Astra camera"
ASTRA_SCRIPTS="${REPO_DIR}/src/ros2_astra_camera/astra_camera/scripts"
bash "${ASTRA_SCRIPTS}/install.sh"
udevadm control --reload-rules
udevadm trigger
ok "Astra udev rules installed via install.sh → /etc/udev/rules.d/56-orbbec-usb.rules"

# =============================================================================
# 13. rosdep install (resolve any remaining ROS deps from package.xml)
# =============================================================================
step "rosdep install"
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo -u "${ROVER_USER}" bash -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${WS_DIR}
    rosdep install --from-paths ${REPO_DIR}/src --ignore-src -r -y
"
ok "rosdep dependencies resolved"

# =============================================================================
# 14. Build workspace
# =============================================================================
step "colcon build  (this takes 5–15 min on a Pi)"
sudo -u "${ROVER_USER}" bash -c "
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${WS_DIR}
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1 | tee ${WS_DIR}/build.log
"
ok "Workspace built → ${WS_DIR}/install"

# =============================================================================
# 15. Shell environment — auto-source ROS in .bashrc
# =============================================================================
step "Shell environment"

BASHRC="${ROVER_HOME}/.bashrc"
ROS_SETUP_LINE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP_LINE="source ${WS_DIR}/install/setup.bash"
DOMAIN_LINE="export ROS_DOMAIN_ID=0"

add_to_bashrc() {
    local line="$1"
    grep -qxF "${line}" "${BASHRC}" || echo "${line}" >> "${BASHRC}"
}

add_to_bashrc "${ROS_SETUP_LINE}"
add_to_bashrc "${WS_SETUP_LINE}"
add_to_bashrc "${DOMAIN_LINE}"
add_to_bashrc "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

chown "${ROVER_USER}:${ROVER_USER}" "${BASHRC}"
ok ".bashrc updated with ROS source lines"

# =============================================================================
# 16. systemd service — auto-start controller_server on boot
# =============================================================================
step "systemd — re-rassor-controller.service"

# Launch script run by systemd
LAUNCH_SCRIPT="/usr/local/bin/re_rassor_start.sh"
cat > "${LAUNCH_SCRIPT}" << EOF
#!/usr/bin/env bash
# RE-RASSOR controller server launcher — called by systemd
# Waits for network before starting

set -euo pipefail

sleep 10   # allow udev, USB enumeration, and network to settle

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WS_DIR}/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 run re_rassor_controller_server controller_server
EOF
chmod +x "${LAUNCH_SCRIPT}"
ok "Launch script → ${LAUNCH_SCRIPT}"

# systemd unit
cat > /etc/systemd/system/re-rassor-controller.service << EOF
[Unit]
Description=RE-RASSOR Controller Server (Flask + ROS2)
Documentation=https://github.com/UCF-Team22-Senior-Design/2025-Fall-UCF-RE-RASSOR-System-Autonomy
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${ROVER_USER}
Group=${ROVER_USER}
ExecStart=${LAUNCH_SCRIPT}
Restart=on-failure
RestartSec=10s
# Give ROS2 DDS time to clean up between restarts
TimeoutStopSec=15

# Capture logs — view with: journalctl -u re-rassor-controller -f
StandardOutput=journal
StandardError=journal
SyslogIdentifier=re-rassor

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable re-rassor-controller.service
ok "re-rassor-controller.service enabled — starts on boot"

# =============================================================================
# 17. Optional: swap / performance tuning for Pi
# =============================================================================
step "Pi performance tuning"

# Increase swap to 2 GB — colcon + Nav2 can exhaust RAM on Pi 4 (4 GB)
DPHYS="/etc/dphys-swapfile"
if [[ -f "${DPHYS}" ]]; then
    if ! grep -q "CONF_SWAPSIZE=2048" "${DPHYS}"; then
        sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' "${DPHYS}"
        dphys-swapfile setup && dphys-swapfile swapon || true
        ok "Swap increased to 2 GB"
    else
        ok "Swap already 2 GB"
    fi
else
    warn "dphys-swapfile not found — install it if colcon runs out of memory:"
    warn "  sudo apt install dphys-swapfile"
fi

# GPU memory split — give more RAM to CPU (rover has no display)
BOOT_CONFIG="/boot/firmware/config.txt"
if [[ -f "${BOOT_CONFIG}" ]]; then
    if ! grep -q "^gpu_mem=" "${BOOT_CONFIG}"; then
        echo "gpu_mem=16" >> "${BOOT_CONFIG}"
        ok "GPU memory reduced to 16 MB (more RAM for ROS2)"
    else
        ok "gpu_mem already set in ${BOOT_CONFIG}"
    fi
fi

# =============================================================================
# Done
# =============================================================================
echo -e "\n${GREEN}${BOLD}══════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  RE-RASSOR setup complete!${NC}"
echo -e "${GREEN}${BOLD}══════════════════════════════════════════${NC}"

echo -e "
${BOLD}Next steps:${NC}

  1. ${YELLOW}Verify Arduino USB port IDs${NC} (physical port matters for udev):
       udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
     Expected:  wheel → ${WHEEL_USB_KERNEL}   drum → ${DRUM_USB_KERNEL}
     If different, edit: /etc/udev/rules.d/01-arduino.rules

  2. ${YELLOW}Reboot${NC} to apply group memberships and udev rules:
       sudo reboot

  3. ${YELLOW}After reboot, verify devices exist${NC}:
       ls -la /dev/arduino_wheel /dev/arduino_drum /dev/astra*

  4. ${YELLOW}Start manually (first time, to see logs)${NC}:
       ros2 run re_rassor_controller_server controller_server

  5. ${YELLOW}Or start via systemd${NC}:
       sudo systemctl start re-rassor-controller
       journalctl -u re-rassor-controller -f

  6. ${YELLOW}Connect the controller app${NC} to http://<rover-ip>:5000

${BOLD}Logs:${NC}
  Build log   : ${WS_DIR}/build.log
  Runtime log : journalctl -u re-rassor-controller -f
"
