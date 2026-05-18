# gz_depth_republisher

C++ ROS 2 Humble package that subscribes to a Gazebo `gz.msgs.Image` depth
topic + `gz.msgs.CameraInfo`, back-projects every valid pixel through the
pinhole camera model, and publishes a `sensor_msgs/PointCloud2` for
`octomap_server`.

---

## Why depth Image instead of PointCloudPacked?

| | PointCloudPacked | **Depth Image** (this package) |
|---|---|---|
| Per-point parsing | Runtime field-offset lookup | Simple flat buffer — 1 scalar per pixel |
| Invalid pixel check | Struct decode → NaN/zero check | Direct value check before any projection |
| Intrinsics dependency | Baked into cloud | Explicit — you control the projection |
| Ghost pixel fix | Filter after decode | Filter **before** projection |

---

## How it works

```
gz.msgs.CameraInfo  ──►  store fx, fy, cx, cy
gz.msgs.Image       ──►  for each pixel:
                           1. read depth  (FLOAT32 / FLOAT64 / L_INT16)
                           2. drop if 0, NaN, Inf, < min_range, > max_range
                           3. back-project:
                                X = (col - cx) * depth / fx
                                Y = (row - cy) * depth / fy
                                Z = depth
                           4. second distance check after projection
                         publish PointCloud2 (XYZ FLOAT32, height=1, is_dense=true)
```

---

## Supported pixel formats

| gz PixelFormatType | Value | Interpretation |
|---|---|---|
| `FLOAT32` | 16 | 4 B/px, depth in **metres** (most common) |
| `FLOAT64` | 17 | 8 B/px, depth in **metres** |
| `L_INT16`  | 6  | 2 B/px, depth in **millimetres** |

---

## Dependencies

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# gz C++ dev libs — pick the version matching your Gazebo release
# Gazebo Garden (default on Humble):
sudo apt install libgz-transport12-dev libgz-msgs9-dev

# Gazebo Harmonic:
sudo apt install libgz-transport13-dev libgz-msgs10-dev
```

---

## Build

```bash
cd ~/ros2_ws/src
cp -r /path/to/gz_depth_republisher .

cd ~/ros2_ws
colcon build --packages-select gz_depth_republisher \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Run

```bash
# defaults
ros2 launch gz_depth_republisher gz_depth_republisher.launch.py

# custom topics
ros2 launch gz_depth_republisher gz_depth_republisher.launch.py \
    gz_image_topic:=/rgbd_camera/depth_image \
    gz_info_topic:=/rgbd_camera/camera_info \
    ros_topic:=/cloud_in \
    min_range:=0.2 \
    max_range:=8.0
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `gz_image_topic` | `/depth_camera/depth_image` | gz Image input |
| `gz_info_topic` | `/depth_camera/camera_info` | gz CameraInfo input |
| `ros_topic` | `/depth_camera/points_filtered` | ROS 2 PointCloud2 output |
| `min_range` | `0.3` | Near-field cutoff metres |
| `max_range` | `10.0` | Far-field cutoff metres |
| `frame_id` | `camera_depth_optical_frame` | Header frame |

---

## Find your gz topic names

```bash
gz topic -l | grep -E "depth|camera"
```

---

## Wire up to octomap_server

```bash
ros2 run octomap_server octomap_server_node \
    --ros-args -p frame_id:=map -p resolution:=0.05 \
    --remap cloud_in:=/depth_camera/points_filtered
```
