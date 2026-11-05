# Lunabotics OAK-D Pro + RTAB-Map

Mapping stack for an OAK-D Pro with RTAB-Map, Nav2, AprilTag and Foxglove on ROS 2 Humble.

## Install

Requires Ubuntu 22.04 with ROS 2 Humble and the Ubuntu Desktop terminal (`gnome-terminal`).

```bash
./install.sh
```

Then replug the camera and open a new terminal. `install.sh` copies the files to `~/` because that's
where `mapping.sh` looks for them, so rerun it after changing a file in this repo. If you edit the
copies in `~/`, copy those changes back here too.

## Run

```bash
~/mapping.sh                                   # handheld test, new map
HANDHELD=false CAM_Z=<height_m> ~/mapping.sh   # camera mounted on the rover
MAP=arena ~/mapping.sh                         # build ~/maps/arena.db (wipes that map first)
MAP=arena FRESH=false ~/mapping.sh             # keep adding to ~/maps/arena.db
MAP=arena LOCALIZE=true ~/mapping.sh           # drive on a saved map without changing it
VIZ=false ~/mapping.sh                         # no rtabmap_viz GUI
```

`mapping.sh` stops any mapping stack already running, including Nav2 and any ROS component container.

## Files

| File | Purpose |
|---|---|
| `mapping.sh` | Starts everything as tabs in one terminal window |
| `rtabmap_fast.launch.py` | Camera, IMU filter, odometry and SLAM nodes, with RTAB-Map parameters |
| `oak_fast.yaml` | Camera settings |
| `fastdds_shm.xml` | Sends images over shared memory to avoid dropped frames |
| `rtabmapGUI.ini` | rtabmap_viz view settings (`__HOME__` is filled in on install) |

Not included: `navgoal_apriltag.py`. `mapping.sh` skips the Tag nav tab if `~/navgoal_apriltag.py` is missing.

## Reference results

Measured on an i7-8665U laptop with the full stack, using 720P stereo, before the switch to `400P`:
RGB and depth at 29.4 Hz, 69 ms frame age, odometry at 27.9 Hz. To compare on another machine:

```bash
ros2 topic hz /oak/stereo/image_raw
ros2 topic hz /odom
```

Testing so far has been handheld, not on the rover.

Package versions on the machine this was tuned on:

| Package | Version |
|---|---|
| `ros-humble-depthai-ros` | 2.12.2 |
| `ros-humble-rtabmap-ros` | 0.23.13 |
| `ros-humble-imu-filter-madgwick` | 2.1.6 |
| `ros-humble-nav2-bringup` | 1.1.20 |
| `ros-humble-apriltag-ros` | 3.4.0 |
| `ros-humble-foxglove-bridge` | 3.6.0 |
