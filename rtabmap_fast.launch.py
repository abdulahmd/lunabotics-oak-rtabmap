#!/usr/bin/python3
# Optimized RTAB-Map launch for OAK-D Pro on a turning rover (Lunabotics).
#
# Differences vs depthai_ros_driver/rtabmap.launch.py:
#  - odometry and SLAM run as separate processes (the stock launch loads them into the
#    camera's single-threaded container, so camera + odom + SLAM all shared one CPU thread)
#  - 640x360 @ 30 fps images (see ~/oak_fast.yaml) instead of 720p @ 10 fps
#  - IMU (through a Madgwick filter) feeds odometry so fast turns don't lose tracking
#  - odometry auto-recovers when lost instead of staying red forever
#  - proper TF tree: map -> odom -> base_link -> oak
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    use_imu = LaunchConfiguration("use_imu").perform(context) == "true"
    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    common = {
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "subscribe_rgb": True,
        "subscribe_depth": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.0,
        "topic_queue_size": 10,
        "sync_queue_size": 10,
        "wait_imu_to_init": use_imu,
        # Full 6DoF: the rover pitches/rolls on regolith; forcing 3DoF lost tracking more
        # in testing. The IMU keeps it gravity-aligned.
        "Reg/Force3DoF": "false",
        "Rtabmap/ImagesAlreadyRectified": "false",
    }

    odom_params = dict(common)
    odom_params.update({
        "publish_tf": True,
        "Odom/Strategy": "0",           # frame-to-map (local feature map survives turns better)
        # Reset only after 30 lost frames in a row (about 1 s). At 1, every single bad frame threw
        # the local map away and froze the pose while the camera kept moving: a real run had 33
        # resets in 58 s and up to 60 deg of drift, which drew the same surfaces in several places.
        "Odom/ResetCountdown": "30",
        "Odom/GuessMotion": "true",
        # Fast turns move features further between frames: 90 deg/s at 20 Hz is about 40 px, the
        # default window. Widen it so matches aren't lost mid-turn.
        "Vis/CorGuessWinSize": "80",
        # The RGB sensor is rolling shutter, so turning skews the image by more than the default
        # 2 px allowance and good matches get rejected as outliers.
        "Vis/PnPReprojError": "3",
        "Vis/FeatureType": "6",         # GFTT/BRIEF: fast
        # Feature matching. (Optical flow, 1, is not supported by frame-to-map odometry:
        # RTAB-Map silently fell back to 0 anyway.)
        "Vis/CorType": "0",
        "Vis/MaxFeatures": "1000",
        "GFTT/MinDistance": "5",        # spread features across the image
        "GFTT/QualityLevel": "0.001",
        "Vis/DepthAsMask": "false",     # also use features without valid depth (walls, far away)
        # Cap depth: OAK stereo depth beyond ~6 m is noisy and was pulling the 6DoF
        # estimate into large fake roll/pitch swings
        "Vis/MaxDepth": "6.0",
        # Last map had single-frame odometry jumps of up to 1 m / 15 deg (accepted on weak
        # matches). A wrong pose smears the map; a lost frame just auto-resets, so require
        # more inliers and a tighter 3D inlier distance.
        "Vis/MinInliers": "15",
        "Vis/InlierDistance": "0.05",
        "OdomF2M/MaxSize": "2000",
        "OdomF2M/BundleAdjustment": "0",  # local BA was too slow; skipping it lets odom keep up with 30 fps
    })

    slam_params = dict(common)
    slam_params.update({
        "subscribe_odom_info": True,
        # Rover (camera bolted on, flat ground): map graph is x, y, yaw only so odometry
        # roll/pitch drift can't tilt each keyframe's cloud. Handheld: must be false, or every
        # cloud is drawn level while the camera was really tilted, smearing the map.
        "Reg/Force3DoF": LaunchConfiguration("force_3dof").perform(context),
        "Rtabmap/DetectionRate": "2.0",  # map updates per second; each update takes ~50 ms
        "RGBD/LinearUpdate": "0.1",
        "RGBD/AngularUpdate": "0.1",
        "Mem/STMSize": "10",
        "Mem/NotLinkedNodesKept": "false",
        "Vis/MaxDepth": "5.0",
        # --- Loop closure: snap revisited places back onto the first pass ---
        # More visual words per node -> stronger place recognition when returning
        "Kp/MaxFeatures": "500",
        "Vis/MaxFeatures": "1000",
        # Accept a loop closure with 15 matched 3D features instead of 20; the
        # OptimizeMaxError check below still rejects wrong ones
        "Vis/MinInliers": "15",
        # Proximity detection: when the rover comes back near an old node, register
        # against it directly. Default only looks 50 nodes back along the graph (~5 m at
        # 0.1 m per node), so returning after a longer loop was never checked. 0 = no limit.
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/ProximityAngle": "60",   # allow a bit more heading difference on return
        # Reject a loop closure if it bends the graph more than 3x the expected error
        # (default; guards against false matches on repetitive regolith)
        "RGBD/OptimizeMaxError": "3.0",
        "Optimizer/Iterations": "100",
        # Refine each accepted loop closure with a small bundle adjustment over the
        # neighbouring nodes: more accurate snap when a revisited place is recognized
        "RGBD/LocalBundleOnLoopClosure": "true",
        # Re-register consecutive map nodes so odometry drift doesn't smear surfaces
        "RGBD/NeighborLinkRefining": "true",
        "Grid/Sensor": "1",  # occupancy grid from depth image
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "4.0",
        "Grid/DepthDecimation": "4",
        # Drop isolated points (stereo noise) from the map cloud
        "Grid/NoiseFilteringRadius": "0.1",
        "Grid/NoiseFilteringMinNeighbors": "5",
        "Grid/CellSize": "0.05",
        "Grid/MaxObstacleHeight": "1.0",
        "Grid/RayTracing": "true",
        # Lunabotics arena: ground steeper than 25 deg (rock faces, crater rims) is an
        # obstacle for the rover; default 45 let rock sides be marked as drivable ground
        "Grid/MaxGroundAngle": "25",
        # Ignore tiny obstacle blobs (stereo speckle) so the 2D grid isn't peppered
        "Grid/MinClusterSize": "20",
    })

    if LaunchConfiguration("localization").perform(context) == "true":
        # Re-use a saved arena map without adding to it: the rover localizes against it
        slam_params.update({
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
        })
    slam_params["database_path"] = os.path.expanduser(
        LaunchConfiguration("database_path").perform(context))

    remappings = [
        ("rgb/image", f"{name}/rgb/image_raw"),
        ("rgb/camera_info", f"{name}/rgb/camera_info"),
        ("depth/image", f"{name}/stereo/image_raw"),
        ("imu", "/imu/data"),
    ]

    fresh = (LaunchConfiguration("fresh_map").perform(context) == "true"
             and slam_params.get("Mem/IncrementalMemory") != "false")
    slam_args = ["-d"] if fresh else []

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": LaunchConfiguration("params_file"),
                # Camera URDF hangs off base_link, so TF is map->odom->base_link->oak
                "parent_frame": "base_link",
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                # RTAB-Map undistorts the raw image itself (Rtabmap/ImagesAlreadyRectified=false),
                # cheaper than the image_proc rectify hop
                "rectify_rgb": "false",
            }.items(),
        ),
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[odom_params],
            remappings=remappings,
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[slam_params],
            remappings=remappings,
            arguments=slam_args,
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("viz")),
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=[slam_params],
            remappings=remappings,
        ),
    ]

    if use_imu:
        # OAK-D Pro IMU only gives raw accel/gyro; Madgwick adds the orientation RTAB-Map needs.
        actions.append(
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[{"use_mag": False, "world_frame": "enu", "publish_tf": False}],
                remappings=[("imu/data_raw", f"{name}/imu/data")],
            )
        )

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(os.path.expanduser("~"), "oak_fast.yaml"),
        ),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("viz", default_value="true"),
        DeclareLaunchArgument("fresh_map", default_value="false"),
        # true = localize on the saved map in database_path (never deleted, never extended)
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap.db"),
        # true = rover (2D map graph); false = handheld camera testing
        DeclareLaunchArgument("force_3dof", default_value="true"),
        # Camera mounting position relative to base_link (meters / radians)
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
