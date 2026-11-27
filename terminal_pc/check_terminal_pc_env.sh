#!/usr/bin/env bash
# READ-ONLY check of the Terminal PC before moving the HMD/SteamVR/VIVE there (terminal_pc/README.md).
# Changes nothing. Prints a report and saves it to /tmp/rhps1_terminal_pc_check_<time>.txt.
#
#   bash terminal_pc/check_terminal_pc_env.sh
OUT="/tmp/rhps1_terminal_pc_check_$(date +%Y%m%d_%H%M%S).txt"
ROUTER="${RHPS1_ZENOH_ROUTER:-10.16.236.199}"
OPERATOR_IP="${RHPS1_OPERATOR_PC_IP:-}"

section() { printf '\n==== %s ====\n' "$1"; }
have() { command -v "$1" >/dev/null 2>&1; }

{
section "host / OS"
hostname; hostname -I 2>/dev/null; lsb_release -ds 2>/dev/null || grep PRETTY /etc/os-release; uname -r

section "GPUs"
lspci 2>/dev/null | grep -iE "vga|3d|display"
have nvidia-smi && nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null

section "display connectors (the VIVE Pro needs a DisplayPort on the GPU that should run SteamVR)"
for c in /sys/class/drm/card*-*; do
  [[ -e "$c/status" ]] || continue
  card="${c##*/}"; card="${card%%-*}"
  drv="$(basename "$(readlink -f "/sys/class/drm/$card/device/driver")" 2>/dev/null)"
  echo "$(basename "$c")  $(cat "$c/status")  driver=$drv"
done

section "Vulkan devices"
if have vulkaninfo; then vulkaninfo --summary 2>/dev/null | grep -E "deviceName|driverName|driverInfo|apiVersion"; else echo "vulkaninfo not installed (package vulkan-tools)"; fi

section "ROS 2"
ls -d /opt/ros/* 2>/dev/null
for pkg in rmw_zenoh_cpp rmw_cyclonedds_cpp; do
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    (source /opt/ros/humble/setup.bash >/dev/null 2>&1; ros2 pkg prefix "$pkg" >/dev/null 2>&1 && echo "$pkg: yes" || echo "$pkg: NO")
  fi
done
python3 -c "import cv2, numpy; print('python3 cv2', cv2.__version__, 'numpy', numpy.__version__)" 2>&1 | tail -1

section "SteamVR / OpenVR"
for d in "$HOME/.local/share/Steam" "$HOME/.steam/steam"; do
  [[ -d "$d/steamapps/common/SteamVR" ]] && echo "SteamVR: $d/steamapps/common/SteamVR"
done
for f in "$HOME/.local/share/Steam/config/steamvr.vrsettings" "$HOME/.steam/steam/config/steamvr.vrsettings"; do
  [[ -f "$f" ]] && { echo "settings: $f"; grep -E "turnOffScreensTimeout|forceFadeOnBadTracking|disableLinuxWaitForPresent" "$f"; }
done
ls "$HOME/openvr/headers/openvr.h" "$HOME/openvr/bin/linux64/libopenvr_api.so" 2>&1

section "VIVE interface (ViveSimpleInterface / vr_interface_node)"
found=0; for d in "$HOME"/luyisi/ViveSimpleInterface* "$HOME"/ViveSimpleInterface*; do [[ -d "$d" ]] && { echo "$d"; found=1; }; done
[[ $found == 1 ]] || echo "not found under ~/ or ~/luyisi"
ls "$HOME"/luyisi/ViveSimpleInterface-main/ros/install/setup.bash 2>/dev/null

section "build tools"
for t in git cmake g++; do printf '%s: ' "$t"; have "$t" && "$t" --version 2>/dev/null | head -1 || echo "missing"; done
dpkg -l 2>/dev/null | awk '/^ii/ && ($2 ~ /^(libvulkan-dev|libopencv-dev|libsdl2-dev|libglew-dev)(:amd64)?$/){print $2, $3}'

section "network"
printf 'Zenoh router %s:7447 ... ' "$ROUTER"; timeout 3 bash -c "echo > /dev/tcp/$ROUTER/7447" 2>/dev/null && echo reachable || echo NOT reachable
[[ -n "$OPERATOR_IP" ]] && { printf 'Operator PC %s ping ... ' "$OPERATOR_IP"; ping -c1 -W2 "$OPERATOR_IP" >/dev/null 2>&1 && echo ok || echo no reply; }
printf 'ufw firewall: '; grep -h '^ENABLED=' /etc/ufw/ufw.conf 2>/dev/null || echo unknown
printf 'TCP port 30555 in use: '; (ss -ltn 2>/dev/null | grep -q ':30555 ') && echo yes || echo no

section "net.core.rmem_max"
sysctl -n net.core.rmem_max 2>/dev/null

section "disk"
df -h "$HOME" | tail -1
} 2>&1 | tee "$OUT"
echo
echo "Saved: $OUT  (send this file or its content)"
