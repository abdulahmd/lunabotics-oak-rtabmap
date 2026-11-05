#!/bin/bash

# Options (e.g. VIZ=false FRESH=true ./mapping.sh)
VIZ=${VIZ:-true}        # rtabmap_viz GUI; set false on the rover to save CPU (use Foxglove instead)
FRESH=${FRESH:-true}   # true = delete ~/.ros/rtabmap.db and start a new map
HANDHELD=${HANDHELD:-true}  # true = camera held in hand (full 3D map); false = bolted on rover (flat 2D map)
if [ "$HANDHELD" = "true" ]; then FORCE_3DOF=false; else FORCE_3DOF=true; fi
# Saved maps: map the arena once, then drive on it, e.g.
#   MAP=arena ./mapping.sh                  -> builds ~/maps/arena.db (FRESH=true wipes it first)
#   MAP=arena LOCALIZE=true ./mapping.sh    -> localizes on ~/maps/arena.db without changing it
MAP=${MAP:-}
LOCALIZE=${LOCALIZE:-false}
if [ -n "$MAP" ]; then mkdir -p ~/maps; DB=~/maps/$MAP.db; else DB=~/.ros/rtabmap.db; fi
if [ "$LOCALIZE" = "true" ] && [ ! -f "$DB" ]; then echo "No saved map at $DB"; exit 1; fi
# Camera mount position relative to base_link (meters / radians).
# The map's floor (z=0, where the occupancy grid is drawn) is base_link's starting height, so
# CAM_Z=0.3048 puts the grid 1 ft below the camera. On the rover set CAM_Z to the real camera height.
CAM_X=${CAM_X:-0.0}; CAM_Y=${CAM_Y:-0.0}; CAM_Z=${CAM_Z:-0.3048}; CAM_PITCH=${CAM_PITCH:-0.0}

# All processes open as tabs in ONE terminal window (gnome-terminal is called once at the end).
# tab "Title" "command": the tab stays open (exec bash) after the command exits.
TABS=()
# Every process uses ~/fastdds_shm.xml: images go over a large shared-memory segment, which stopped
# the default transport dropping RGB frames (13 -> 18 Hz with depth at 18 Hz in testing).
tab() { TABS+=(--tab --title="$1" -e "bash -c $(printf %q "export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_shm.xml; $2; exec bash")"); }

# Laptop "powersave" governor throttles the CPU and slows odometry
powerprofilesctl set performance 2>/dev/null

# Stop any stack left over from a previous run (closing the terminal window doesn't kill it).
# Two copies at once fight over the camera, TF and rtabmap.db and scramble the map.
# SIGINT first so RTAB-Map saves its database cleanly.
STACK='rtabmap_fast.launch.py|navigation_launch.py|rtabmap_slam/rtabmap|rtabmap_viz/rtabmap_viz|rtabmap_odom/rgbd_odometry|imu_filter_madgwick_node|oak_container|apriltag_node|foxglove_bridge|navgoal_apriltag'
if pgrep -f "$STACK" >/dev/null; then
  echo "Stopping previous mapping stack..."
  pkill -INT -f "$STACK"
  for i in $(seq 20); do pgrep -f "$STACK" >/dev/null || break; sleep 0.5; done
  pkill -KILL -f "$STACK" 2>/dev/null
  pkill -KILL -f 'nav2_|lifecycle_manager|component_container' 2>/dev/null
fi
# Wait loops left in old terminal tabs would otherwise fire into the new session
pkill -f 'until ros2 ' 2>/dev/null

# Each step waits for what it depends on (polls every 1 s) instead of a fixed sleep,
# so a fast start isn't held up and a slow start (camera boot, big DB) doesn't break anything.
WAIT_MAP="until ros2 topic info /map 2>/dev/null | grep -q 'Publisher count: [1-9]'; do sleep 1; done"
WAIT_NAV="until ros2 action list 2>/dev/null | grep -qx /navigate_to_pose; do sleep 1; done"

# 1. Launch RTAB-Map (optimized: ~/rtabmap_fast.launch.py + ~/oak_fast.yaml)
tab "RTAB-Map" "ros2 launch ~/rtabmap_fast.launch.py viz:=$VIZ fresh_map:=$FRESH localization:=$LOCALIZE database_path:=$DB force_3dof:=$FORCE_3DOF cam_pos_x:=$CAM_X cam_pos_y:=$CAM_Y cam_pos_z:=$CAM_Z cam_pitch:=$CAM_PITCH"

# (base_link -> oak TF comes from the launch file's camera URDF; don't add a static_transform_publisher.)
# Odometry and the map already start at zero, so no reset or initial pose step is needed.

# 2. Nav2 Bringup, once RTAB-Map is publishing /map
tab "Nav2" "$WAIT_MAP; ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false"

# 3. AprilTag Node
tab "AprilTag" "ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/oak/rgb/image_raw -r camera_info:=/oak/rgb/camera_info"

# 4. Foxglove Bridge
tab "Foxglove" "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

# 5. Apriltag Navigation Python Script, once Nav2 accepts goals (skipped if the script is missing)
if [ -f ~/navgoal_apriltag.py ]; then
  tab "Tag nav" "$WAIT_NAV; while true; do python3 ~/navgoal_apriltag.py 6; echo 'Scan complete. Press Enter to scan for Tag 6 again, or Ctrl+C to quit.'; read; done"
else
  echo "Note: ~/navgoal_apriltag.py not found, skipping Tag nav tab"
fi

# Additional Commands
# Delete Memory
# gnome-terminal -- bash -c "sleep 5; ros2 service call /rtabmap/reset std_srvs/srv/Empty {}; exec bash"

# Open the window with all tabs (-e is deprecated but is the only way to give each tab its own command)
gnome-terminal --window "${TABS[@]}" 2>/dev/null
