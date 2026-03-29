#!/usr/bin/env bash
# =============================================================================
# RE-RASSOR Pi Image Builder
# Creates a plug-and-play Raspberry Pi SD card image (.img) containing
# the full RE-RASSOR rover workspace, drivers, and first-boot automation.
#
# Flash the output image to a microSD card, insert into Pi → rover is ready.
#
# Usage:
#   sudo bash make_pi_image.sh [OPTIONS]
#
# Options:
#   --size GB          Image size in GB           (default: 32)
#   --output FILE      Output image filename       (default: re-rassor-pi-YYYY-MM-DD.img)
#   --wifi-ssid SSID   Pre-configure WiFi SSID
#   --wifi-pass PASS   Pre-configure WiFi password
#   --skip-download    Use cached base image if present
#   --compress         xz-compress the final image (produces .img.xz)
#   -h|--help          Show this help
#
# Requirements (installed automatically if missing):
#   xz-utils parted e2fsprogs rsync
#
# Notes:
#   • The Pi needs internet access on first boot to install ROS2 + system packages.
#   • First-boot build takes 5–15 min on a Pi 4. Monitor via:
#       sudo journalctl -u re-rassor-firstboot -f
#   • If running on aarch64 (ARM) and build/ + install/ exist in the workspace,
#     they are embedded in the image to skip the colcon build on first boot.
# =============================================================================
set -euo pipefail

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"   # /home/mistry/ros2_ws (or equivalent)
REPO_DIR="${SCRIPT_DIR}"                     # this repo's root

ROS_DISTRO="jazzy"
ROVER_USER="ubuntu"
ROVER_HOME="/home/${ROVER_USER}"
PI_WS_DIR="${ROVER_HOME}/ros2_ws"
PI_REPO_DIR="${PI_WS_DIR}/2025-Fall-UCF-RE-RASSOR-System-Autonomy"

# Ubuntu 24.04 LTS ARM64 Raspberry Pi preinstalled server image
BASE_IMAGE_URL="https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.2-preinstalled-server-arm64+raspi.img.xz"
BASE_IMAGE_XZ="$(basename "${BASE_IMAGE_URL}")"
BASE_IMAGE_IMG="${BASE_IMAGE_XZ%.xz}"

# Arduino physical USB port kernel IDs (edit if hardware layout differs)
WHEEL_USB_KERNEL="1-2"
DRUM_USB_KERNEL="1-1.1"

# Default login credentials baked into the image
DEFAULT_PASS="rassor"

# ── Defaults (overridden by CLI args) ─────────────────────────────────────────
IMAGE_SIZE_GB=32
OUTPUT_IMAGE=""
WIFI_SSID=""
WIFI_PASS=""
SKIP_DOWNLOAD=false
COMPRESS=false

# ── Colour helpers ─────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

step()  { echo -e "\n${CYAN}${BOLD}▶ $*${NC}"; }
ok()    { echo -e "  ${GREEN}✔ $*${NC}"; }
warn()  { echo -e "  ${YELLOW}⚠ $*${NC}"; }
die()   { echo -e "\n${RED}✖ $*${NC}" >&2; exit 1; }
info()  { echo -e "  ${BOLD}→ $*${NC}"; }

# ── Argument parsing ───────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --size)          IMAGE_SIZE_GB="$2"; shift 2 ;;
        --output)        OUTPUT_IMAGE="$2";  shift 2 ;;
        --wifi-ssid)     WIFI_SSID="$2";     shift 2 ;;
        --wifi-pass)     WIFI_PASS="$2";     shift 2 ;;
        --skip-download) SKIP_DOWNLOAD=true; shift   ;;
        --compress)      COMPRESS=true;      shift   ;;
        -h|--help)
            grep '^# ' "$0" | head -30 | sed 's/^# //'
            exit 0 ;;
        *) die "Unknown option: $1  (run with --help for usage)" ;;
    esac
done

[[ -z "${OUTPUT_IMAGE}" ]] && OUTPUT_IMAGE="re-rassor-pi-$(date +%Y-%m-%d).img"

# ── Guards ─────────────────────────────────────────────────────────────────────
[[ $EUID -eq 0 ]] || die "Run with sudo:  sudo bash make_pi_image.sh"

HOST_ARCH=$(uname -m)

echo -e "\n${BOLD}RE-RASSOR Pi Image Builder${NC}"
echo "  Repo    : ${REPO_DIR}"
echo "  Output  : ${OUTPUT_IMAGE}"
echo "  Size    : ${IMAGE_SIZE_GB} GB"
echo "  Host    : ${HOST_ARCH}"
[[ -n "${WIFI_SSID}" ]] && echo "  WiFi    : ${WIFI_SSID}"

# ── Dependency check ───────────────────────────────────────────────────────────
step "Checking host dependencies"
MISSING_PKGS=()
command -v xz        &>/dev/null || MISSING_PKGS+=(xz-utils)
command -v parted    &>/dev/null || MISSING_PKGS+=(parted)
command -v e2fsck    &>/dev/null || MISSING_PKGS+=(e2fsprogs)
command -v resize2fs &>/dev/null || MISSING_PKGS+=(e2fsprogs)
command -v rsync     &>/dev/null || MISSING_PKGS+=(rsync)
command -v openssl   &>/dev/null || MISSING_PKGS+=(openssl)

if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
    info "Installing missing packages: ${MISSING_PKGS[*]}"
    apt-get install -y -qq "${MISSING_PKGS[@]}"
fi
ok "All dependencies satisfied"

# ── Cleanup trap ───────────────────────────────────────────────────────────────
MNT_BOOT=""
MNT_ROOT=""
LOOP_DEV=""
WORK_IMG=""

cleanup() {
    local rc=$?
    set +e
    [[ -n "${MNT_BOOT}" ]] && umount "${MNT_BOOT}" 2>/dev/null; rmdir "${MNT_BOOT}" 2>/dev/null || true
    [[ -n "${MNT_ROOT}" ]] && umount "${MNT_ROOT}" 2>/dev/null; rmdir "${MNT_ROOT}" 2>/dev/null || true
    [[ -n "${LOOP_DEV}" ]] && losetup -d "${LOOP_DEV}" 2>/dev/null || true
    if [[ $rc -ne 0 && -n "${WORK_IMG}" && -f "${WORK_IMG}" ]]; then
        warn "Build failed — removing incomplete image: ${WORK_IMG}"
        rm -f "${WORK_IMG}"
    fi
}
trap cleanup EXIT

# =============================================================================
# 1. Download / locate base Ubuntu Pi image
# =============================================================================
step "Base image (Ubuntu 24.04 ARM64 Raspberry Pi)"

if [[ "${SKIP_DOWNLOAD}" == true && -f "${BASE_IMAGE_IMG}" ]]; then
    ok "Using cached decompressed image: ${BASE_IMAGE_IMG}"
elif [[ "${SKIP_DOWNLOAD}" == true && -f "${BASE_IMAGE_XZ}" ]]; then
    info "Decompressing cached ${BASE_IMAGE_XZ} ..."
    xz -dk "${BASE_IMAGE_XZ}"
    ok "Decompressed → ${BASE_IMAGE_IMG}"
elif [[ -f "${BASE_IMAGE_IMG}" ]]; then
    ok "Found existing image: ${BASE_IMAGE_IMG}"
elif [[ -f "${BASE_IMAGE_XZ}" ]]; then
    info "Decompressing ${BASE_IMAGE_XZ} ..."
    xz -dk "${BASE_IMAGE_XZ}"
    ok "Decompressed → ${BASE_IMAGE_IMG}"
else
    info "Downloading Ubuntu 24.04 ARM64 Pi image (~1.1 GB) ..."
    wget -c --show-progress "${BASE_IMAGE_URL}" -O "${BASE_IMAGE_XZ}"
    info "Decompressing ..."
    xz -dk "${BASE_IMAGE_XZ}"
    ok "Base image ready: ${BASE_IMAGE_IMG}"
fi

# =============================================================================
# 2. Create working copy and expand to target size
# =============================================================================
step "Creating work image and expanding to ${IMAGE_SIZE_GB} GB"

WORK_IMG="${OUTPUT_IMAGE}.work"
info "Copying base image → ${WORK_IMG} ..."
cp --sparse=always "${BASE_IMAGE_IMG}" "${WORK_IMG}"

info "Expanding image file to ${IMAGE_SIZE_GB} GB ..."
truncate -s "${IMAGE_SIZE_GB}G" "${WORK_IMG}"

# Resize partition 2 to fill the new space
info "Resizing partition 2 to fill image ..."
parted "${WORK_IMG}" --script -- resizepart 2 100%
ok "Partition table updated"

# =============================================================================
# 3. Attach loop device and expand filesystem
# =============================================================================
step "Attaching loop device and expanding ext4 filesystem"

LOOP_DEV=$(losetup -fP --show "${WORK_IMG}")
info "Loop device: ${LOOP_DEV}"
info "  ${LOOP_DEV}p1 → /boot/firmware (FAT32)"
info "  ${LOOP_DEV}p2 → / (ext4)"

# e2fsck required before resize2fs
info "Checking filesystem (e2fsck) ..."
e2fsck -f -y "${LOOP_DEV}p2" || true   # exit code 1 = corrected errors (OK)

info "Expanding filesystem (resize2fs) ..."
resize2fs "${LOOP_DEV}p2"
ok "Filesystem expanded to ${IMAGE_SIZE_GB} GB"

# =============================================================================
# 4. Mount partitions
# =============================================================================
step "Mounting image partitions"

MNT_BOOT=$(mktemp -d /tmp/re-rassor-boot.XXXXXX)
MNT_ROOT=$(mktemp -d /tmp/re-rassor-root.XXXXXX)

mount "${LOOP_DEV}p1" "${MNT_BOOT}"
mount "${LOOP_DEV}p2" "${MNT_ROOT}"
ok "boot → ${MNT_BOOT}"
ok "root → ${MNT_ROOT}"

# =============================================================================
# 5. Inject RE-RASSOR workspace source into image rootfs
# =============================================================================
step "Injecting RE-RASSOR workspace source"

PI_REPO_FULL="${MNT_ROOT}${PI_REPO_DIR}"
mkdir -p "${PI_REPO_FULL}"

info "Copying repo source (src/, setup.sh, scripts ...) ..."
rsync -a --info=progress2 \
    --exclude="*.img"    \
    --exclude="*.img.xz" \
    --exclude="*.img.gz" \
    --exclude="*.work"   \
    --exclude=".cache"   \
    "${REPO_DIR}/" \
    "${PI_REPO_FULL}/"

ok "Workspace source → ${PI_REPO_DIR}"

# =============================================================================
# 6. Inject pre-built binaries (ARM only — x86 binaries won't run on Pi)
# =============================================================================
step "Pre-built binaries"

if [[ "${HOST_ARCH}" == "aarch64" ]]; then
    PREBUILT=false

    if [[ -d "${WS_ROOT}/build" && -d "${WS_ROOT}/install" ]]; then
        info "ARM host + binaries found — embedding build/ and install/ into image ..."
        mkdir -p "${MNT_ROOT}${PI_WS_DIR}"
        rsync -a --info=progress2 "${WS_ROOT}/build/"   "${MNT_ROOT}${PI_WS_DIR}/build/"
        rsync -a --info=progress2 "${WS_ROOT}/install/" "${MNT_ROOT}${PI_WS_DIR}/install/"
        # Sentinel file — firstboot.sh checks this to skip colcon
        touch "${MNT_ROOT}${PI_WS_DIR}/.prebuilt"
        PREBUILT=true
        ok "build/ and install/ embedded — first boot will skip colcon build"
    else
        warn "No build/ or install/ found at ${WS_ROOT} — Pi will build from source on first boot"
    fi
else
    warn "Host is ${HOST_ARCH} (not ARM) — skipping pre-built binaries"
    warn "Pi will compile from source on first boot (5–15 min on Pi 4)"
fi

# =============================================================================
# 7. Install drivers / udev rules directly into image
# =============================================================================
step "Installing drivers and udev rules"

UDEV_DIR="${MNT_ROOT}/etc/udev/rules.d"
mkdir -p "${UDEV_DIR}"

# ── Orbbec Astra camera ───────────────────────────────────────────────────────
cp "${REPO_DIR}/src/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules" \
   "${UDEV_DIR}/56-orbbec-usb.rules"
ok "Astra udev rules    → /etc/udev/rules.d/56-orbbec-usb.rules"

# ── Arduino serial port symlinks ──────────────────────────────────────────────
cat > "${UDEV_DIR}/01-arduino.rules" << UDEVRULES
# RE-RASSOR Arduino serial port symlinks
# KERNELS match the physical USB port position on the Raspberry Pi.
# If /dev/arduino_wheel or /dev/arduino_drum do not appear, run:
#   udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
# then update the KERNELS values below to match your port layout.
SUBSYSTEM=="tty", KERNELS=="${WHEEL_USB_KERNEL}", SYMLINK+="arduino_wheel", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNELS=="${DRUM_USB_KERNEL}",  SYMLINK+="arduino_drum",  MODE="0666", GROUP="dialout"
UDEVRULES

ok "Arduino udev rules  → /etc/udev/rules.d/01-arduino.rules"
warn "Wheel Arduino: USB kernel ${WHEEL_USB_KERNEL} | Drum Arduino: USB kernel ${DRUM_USB_KERNEL}"
warn "Edit /etc/udev/rules.d/01-arduino.rules on the Pi if ports differ"

# =============================================================================
# 8. Create first-boot setup service
# =============================================================================
step "Creating first-boot setup service"

FIRSTBOOT_DIR="${MNT_ROOT}/opt/re_rassor"
mkdir -p "${FIRSTBOOT_DIR}"

# ── First-boot script ─────────────────────────────────────────────────────────
cat > "${FIRSTBOOT_DIR}/firstboot.sh" << FBSCRIPT
#!/usr/bin/env bash
# RE-RASSOR First-Boot Setup Script
# Invoked once by re-rassor-firstboot.service on the Pi's first power-on.
# Runs setup.sh, then reboots.  The sentinel /etc/re-rassor-setup-complete
# prevents re-runs on subsequent boots.
set -euo pipefail

LOG="/var/log/re-rassor-firstboot.log"
exec > >(tee -a "\${LOG}") 2>&1

echo "========================================================"
echo " RE-RASSOR First-Boot Setup"
echo " Started: \$(date)"
echo "========================================================"

ROVER_USER="${ROVER_USER}"
ROVER_HOME="${ROVER_HOME}"
WS_DIR="${PI_WS_DIR}"
REPO_DIR="${PI_REPO_DIR}"

# ── Wait for cloud-init to finish creating the user ──────────────────────────
echo "Waiting for cloud-init to complete user setup ..."
cloud-init status --wait 2>/dev/null || true

# ── Fix ownership of workspace files placed by image builder ─────────────────
echo "Setting workspace ownership → \${ROVER_USER} ..."
chown -R "\${ROVER_USER}:\${ROVER_USER}" "\${WS_DIR}" 2>/dev/null || true

# ── If pre-built ARM binaries are present, skip colcon build ─────────────────
SETUP_ARGS=""
if [[ -f "\${WS_DIR}/.prebuilt" && -d "\${WS_DIR}/install" ]]; then
    echo "Pre-built binaries detected — patching setup to skip colcon build ..."
    # Temporarily stub out the colcon build step only
    SETUP_SCRIPT="\${REPO_DIR}/setup.sh"
    PATCHED_SCRIPT="/tmp/setup_no_build.sh"
    sed '/colcon build/,/ok "Workspace built/s/.*/true/' "\${SETUP_SCRIPT}" > "\${PATCHED_SCRIPT}"
    chmod +x "\${PATCHED_SCRIPT}"
    SETUP_ARGS="\${PATCHED_SCRIPT}"
else
    SETUP_ARGS="\${REPO_DIR}/setup.sh"
fi

# ── Run setup.sh ──────────────────────────────────────────────────────────────
echo "Running setup.sh ..."
export SUDO_USER="\${ROVER_USER}"
bash "\${SETUP_ARGS}"

# ── Mark setup as complete ────────────────────────────────────────────────────
touch /etc/re-rassor-setup-complete
echo "========================================================"
echo " RE-RASSOR First-Boot Setup COMPLETE"
echo " Finished: \$(date)"
echo "========================================================"

# Disable this service so it never runs again
systemctl disable re-rassor-firstboot.service

echo "Rebooting in 10 seconds to apply group memberships and udev rules ..."
sleep 10
systemctl reboot
FBSCRIPT

chmod +x "${FIRSTBOOT_DIR}/firstboot.sh"
ok "First-boot script → /opt/re_rassor/firstboot.sh"

# ── systemd unit ──────────────────────────────────────────────────────────────
cat > "${MNT_ROOT}/etc/systemd/system/re-rassor-firstboot.service" << 'SVCUNIT'
[Unit]
Description=RE-RASSOR First-Boot Setup (runs once)
Documentation=https://github.com/UCF-Team22-Senior-Design/2025-Fall-UCF-RE-RASSOR-System-Autonomy
After=network-online.target cloud-init.target
Wants=network-online.target
# Skip if setup has already completed
ConditionPathExists=!/etc/re-rassor-setup-complete

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/opt/re_rassor/firstboot.sh
StandardOutput=journal+console
StandardError=journal+console
SyslogIdentifier=re-rassor-firstboot
# colcon build on Pi can take up to 30 min — give it plenty of time
TimeoutStartSec=3600

[Install]
WantedBy=multi-user.target
SVCUNIT

ok "First-boot service → /etc/systemd/system/re-rassor-firstboot.service"

# Enable by symlinking into multi-user.target.wants
WANTS_DIR="${MNT_ROOT}/etc/systemd/system/multi-user.target.wants"
mkdir -p "${WANTS_DIR}"
ln -sf "/etc/systemd/system/re-rassor-firstboot.service" \
       "${WANTS_DIR}/re-rassor-firstboot.service"
ok "Service enabled (symlinked into multi-user.target.wants)"

# =============================================================================
# 9. cloud-init: hostname, default user, SSH, packages
# =============================================================================
step "Configuring cloud-init"

# SHA-512 hash of the default password
PASS_HASH=$(echo "${DEFAULT_PASS}" | openssl passwd -6 -stdin)

cat > "${MNT_BOOT}/user-data" << USERDATA
#cloud-config
# RE-RASSOR rover — cloud-init user-data
# Generated by make_pi_image.sh on $(date +%Y-%m-%d)

hostname: re-rassor
manage_etc_hosts: true

# ── Default user ──────────────────────────────────────────────────────────────
users:
  - name: ${ROVER_USER}
    gecos: "RE-RASSOR Rover"
    groups: [sudo, dialout, video, plugdev, adm]
    shell: /bin/bash
    lock_passwd: false
    passwd: ${PASS_HASH}
    sudo: ALL=(ALL) NOPASSWD:ALL

# ── SSH ───────────────────────────────────────────────────────────────────────
ssh_pwauth: true
disable_root: true
chpasswd:
  expire: false

# ── Packages available before first-boot service fires ───────────────────────
# (minimal — setup.sh installs everything else)
packages:
  - git
  - curl
  - wget

package_update: false
package_upgrade: false

# ── Final message ─────────────────────────────────────────────────────────────
final_message: |
  RE-RASSOR Pi image — cloud-init complete.
  First-boot setup will begin shortly.
  Username: ${ROVER_USER}  Password: ${DEFAULT_PASS}
  Monitor: sudo journalctl -u re-rassor-firstboot -f
USERDATA

ok "cloud-init user-data → boot partition"

# cloud-init requires meta-data alongside user-data
cat > "${MNT_BOOT}/meta-data" << 'METADATA'
instance-id: re-rassor-pi-001
local-hostname: re-rassor
METADATA

ok "cloud-init meta-data → boot partition"

# =============================================================================
# 10. Network configuration (Ethernet always on; WiFi optional)
# =============================================================================
step "Network configuration"

if [[ -n "${WIFI_SSID}" ]]; then
    cat > "${MNT_BOOT}/network-config" << NETCFG
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "${WIFI_SSID}":
        password: "${WIFI_PASS}"
NETCFG
    ok "WiFi pre-configured → SSID: ${WIFI_SSID}"
else
    cat > "${MNT_BOOT}/network-config" << 'NETCFG'
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
NETCFG
    info "Ethernet DHCP only (pass --wifi-ssid and --wifi-pass to add WiFi)"
fi

# =============================================================================
# 11. Raspberry Pi boot configuration (config.txt)
# =============================================================================
step "Raspberry Pi boot configuration"

BOOT_CFG="${MNT_BOOT}/config.txt"
if [[ -f "${BOOT_CFG}" ]]; then
    # Reduce GPU memory — rover is headless, more RAM for ROS2
    if ! grep -q "^gpu_mem=" "${BOOT_CFG}"; then
        echo ""                                         >> "${BOOT_CFG}"
        echo "# RE-RASSOR: headless rover optimisation" >> "${BOOT_CFG}"
        echo "gpu_mem=16"                               >> "${BOOT_CFG}"
        ok "gpu_mem=16 → less VRAM, more RAM for ROS2"
    else
        ok "gpu_mem already set in config.txt"
    fi

    # Enable USB host mode (Pi 4 / 5)
    if ! grep -q "^otg_mode=1" "${BOOT_CFG}"; then
        echo "otg_mode=1" >> "${BOOT_CFG}"
        ok "USB host mode enabled (otg_mode=1)"
    fi
else
    warn "config.txt not found at ${BOOT_CFG} — skipping Pi tuning"
fi

# =============================================================================
# 12. Sync, unmount, detach loop device
# =============================================================================
step "Syncing and unmounting"

info "Syncing writes to disk ..."
sync

umount "${MNT_BOOT}"
umount "${MNT_ROOT}"
losetup -d "${LOOP_DEV}"

# Clear trap variables so cleanup() doesn't try to unmount again
MNT_BOOT=""
MNT_ROOT=""
LOOP_DEV=""

info "Renaming work image → ${OUTPUT_IMAGE} ..."
mv "${WORK_IMG}" "${OUTPUT_IMAGE}"
WORK_IMG=""

ok "Image finalised: ${OUTPUT_IMAGE}"

# =============================================================================
# 13. Optional xz compression
# =============================================================================
if [[ "${COMPRESS}" == true ]]; then
    step "Compressing image (xz -T0) — this may take several minutes"
    xz -T0 -v "${OUTPUT_IMAGE}"
    OUTPUT_IMAGE="${OUTPUT_IMAGE}.xz"
    ok "Compressed image: ${OUTPUT_IMAGE}"
fi

# =============================================================================
# Done — print flash instructions
# =============================================================================
IMAGE_SIZE_HUMAN=$(du -sh "${OUTPUT_IMAGE}" | cut -f1)

echo -e "\n${GREEN}${BOLD}══════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  RE-RASSOR Pi Image Ready!${NC}"
echo -e "${GREEN}${BOLD}══════════════════════════════════════════════════════════${NC}"

echo -e "
  Image  : ${BOLD}${OUTPUT_IMAGE}${NC}  (${IMAGE_SIZE_HUMAN})
  Login  : ${BOLD}ubuntu${NC} / ${BOLD}${DEFAULT_PASS}${NC}   ← change after setup!

${BOLD}─── Flash to SD card ──────────────────────────────────────${NC}

  # dd (replace /dev/sdX with your card — check with lsblk):
  sudo dd if=${OUTPUT_IMAGE} of=/dev/sdX bs=4M status=progress conv=fsync

  # Or use Raspberry Pi Imager → \"Use custom image\" → select file above

${BOLD}─── First-boot sequence (fully automatic) ──────────────────${NC}

  1. Insert SD card into Raspberry Pi and power on
  2. cloud-init runs (~1 min)  →  sets hostname, user, SSH
  3. re-rassor-firstboot.service starts and runs setup.sh:
       • apt update / upgrade
       • Installs ROS2 ${ROS_DISTRO} + Nav2 + RTAB-Map
       • Installs Python deps (Flask, pyserial, ...)
       • Builds libuvc from source
       • rosdep install + colcon build  (5–15 min on Pi 4)
       • Enables re-rassor-controller.service (auto-start on boot)
  4. Pi reboots automatically when setup completes
  5. After reboot the rover is fully operational

${BOLD}─── Monitor setup progress ─────────────────────────────────${NC}

  ssh ubuntu@<pi-ip>
  sudo journalctl -u re-rassor-firstboot -f
  # or:
  tail -f /var/log/re-rassor-firstboot.log

${BOLD}─── After setup ────────────────────────────────────────────${NC}

  Controller app   : http://<rover-ip>:5000
  Service status   : sudo systemctl status re-rassor-controller
  Runtime logs     : sudo journalctl -u re-rassor-controller -f

  Verify devices:
    ls -la /dev/arduino_wheel /dev/arduino_drum /dev/astra*
  If Arduino ports differ, update kernel IDs:
    udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
    sudo nano /etc/udev/rules.d/01-arduino.rules
"
