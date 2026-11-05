#!/bin/bash
# Sets up this mapping stack on an Ubuntu 22.04 machine that already has ROS 2 Humble.
# Installs packages and the camera's USB rule, then copies the files to ~/ where mapping.sh expects them.
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble not found at /opt/ros/humble. Install it first:"
  echo "  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
  exit 1
fi

sudo apt update
sudo apt install -y \
  ros-humble-depthai-ros \
  ros-humble-rtabmap-ros \
  ros-humble-imu-filter-madgwick \
  ros-humble-nav2-bringup \
  ros-humble-apriltag-ros \
  ros-humble-foxglove-bridge

# Lets the OAK camera be opened without root. Replug the camera after this runs.
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

# Keep a timestamped copy of anything this would replace
STAMP=$(date +%Y%m%d-%H%M%S)
for f in mapping.sh rtabmap_fast.launch.py oak_fast.yaml fastdds_shm.xml; do
  if [ -f "$HOME/$f" ] && ! cmp -s "$REPO/$f" "$HOME/$f"; then
    cp -p "$HOME/$f" "$HOME/$f.bak-$STAMP"
    echo "Backed up ~/$f to ~/$f.bak-$STAMP"
  fi
  cp "$REPO/$f" "$HOME/$f"
done
chmod +x "$HOME/mapping.sh"

# 3D view settings: only on a first install, so a tuned view on this machine isn't replaced
mkdir -p "$HOME/.ros"
if [ ! -f "$HOME/.ros/rtabmapGUI.ini" ]; then
  sed "s#__HOME__#$HOME#g" "$REPO/rtabmapGUI.ini" > "$HOME/.ros/rtabmapGUI.ini"
fi

# mapping.sh doesn't source ROS itself
if ! grep -qs 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc"; then
  echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
fi

echo
echo "Done. Replug the camera, open a NEW terminal, then run: ~/mapping.sh"
