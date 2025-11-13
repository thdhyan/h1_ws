# H1 Debug Package

This package contains debugging and visualization tools for the H1 humanoid robot project.

## Nodes

### 1. Camera Capture Node

OpenCV-based camera capture node that publishes camera images and camera info.

**Executable:** `camera_capture_node`

**Parameters:**
- `camera_id` (int, default: 0): Camera device ID to capture from
- `output_topic` (string, default: "camera/image_raw"): Topic to publish images
- `camera_info_topic` (string, default: "camera/camera_info"): Topic to publish camera info
- `namespace` (string, default: ""): Optional namespace for topics
- `frame_id` (string, default: "camera_frame"): Frame ID for camera
- `width` (int, default: 640): Desired image width
- `height` (int, default: 480): Desired image height
- `fps` (int, default: 30): Frames per second

**Published Topics:**
- `{namespace}/{output_topic}` (sensor_msgs/Image): Camera images
- `{namespace}/{camera_info_topic}` (sensor_msgs/CameraInfo): Camera calibration info

**Example Usage:**
```bash
ros2 run h1_debug camera_capture_node --ros-args \
  -p camera_id:=0 \
  -p output_topic:=camera/image_raw \
  -p namespace:=robot1
```

### 2. Visualization Grid Node

Creates a 2x2 grid visualization of multiple video streams.

**Executable:** `visualization_grid_node`

**Parameters:**
- `window_name` (string, default: "Visualization Grid"): OpenCV window name
- `cell_width` (int, default: 640): Width of each grid cell
- `cell_height` (int, default: 480): Height of each grid cell
- `show_labels` (bool, default: true): Show frame labels

**Subscribed Topics:**
- `/visualization/frame_1` (sensor_msgs/Image): Top-left frame
- `/visualization/frame_2` (sensor_msgs/Image): Top-right frame
- `/visualization/frame_3` (sensor_msgs/Image): Bottom-left frame
- `/visualization/frame_4` (sensor_msgs/Image): Bottom-right frame

**Example Usage:**
```bash
ros2 run h1_debug visualization_grid_node
```

To publish test frames:
```bash
# Remap camera output to visualization frames
ros2 run h1_debug camera_capture_node --ros-args \
  -r camera/image_raw:=/visualization/frame_1
```

### 3. Pose Detection Node

Runs MediaPipe pose detection on a camera stream.

**Executable:** `pose_detection_node`

**Dependencies:**
```bash
pip install mediapipe
```

**Parameters:**
- `input_topic` (string, default: "camera/image_raw"): Input camera topic
- `output_pose_topic` (string, default: "pose/detection"): Output pose landmarks topic
- `output_image_topic` (string, default: "pose/image_annotated"): Annotated image topic
- `min_detection_confidence` (float, default: 0.5): Minimum detection confidence [0.0-1.0]
- `min_tracking_confidence` (float, default: 0.5): Minimum tracking confidence [0.0-1.0]
- `model_complexity` (int, default: 1): Model complexity (0=lite, 1=full, 2=heavy)
- `enable_segmentation` (bool, default: false): Enable pose segmentation
- `smooth_landmarks` (bool, default: true): Smooth landmark positions

**Subscribed Topics:**
- `{input_topic}` (sensor_msgs/Image): Input camera stream

**Published Topics:**
- `{output_pose_topic}` (h1_msgs/PoseDetection): Detected pose landmarks
- `{output_image_topic}` (sensor_msgs/Image): Annotated image with pose overlay

**Example Usage:**
```bash
ros2 run h1_debug pose_detection_node --ros-args \
  -p input_topic:=camera/image_raw \
  -p min_detection_confidence:=0.7
```

## Launch Files

### debug_nodes.launch.py

Launches all three nodes together for a complete debugging setup.

**Usage:**
```bash
ros2 launch h1_debug debug_nodes.launch.py camera_id:=0
```

## Message Types

The package uses custom messages defined in `h1_msgs`:

### PoseLandmark.msg
```
float32 x          # Normalized x coordinate [0.0-1.0]
float32 y          # Normalized y coordinate [0.0-1.0]
float32 z          # Normalized z coordinate
float32 visibility # Visibility score [0.0-1.0]
float32 presence   # Presence score [0.0-1.0]
```

### PoseDetection.msg
```
std_msgs/Header header
PoseLandmark[] landmarks  # Array of 33 pose landmarks
float32 confidence        # Overall detection confidence
bool detected             # Whether a pose was detected
```

## Building

```bash
cd /path/to/h1_ws
colcon build --packages-select h1_msgs h1_debug
source install/setup.bash
```

## Example Workflow

1. Start camera capture:
```bash
ros2 run h1_debug camera_capture_node
```

2. Start pose detection:
```bash
ros2 run h1_debug pose_detection_node
```

3. View results in visualization grid:
```bash
ros2 run h1_debug visualization_grid_node
```

4. Remap topics to visualization grid:
```bash
ros2 run rqt_image_view rqt_image_view /pose/image_annotated
# Or remap to visualization frame
ros2 run image_transport republish raw in:=/pose/image_annotated raw out:=/visualization/frame_1
```

## Dependencies

- ROS 2 (Humble or later recommended)
- Python 3
- OpenCV (python3-opencv)
- cv_bridge
- MediaPipe (for pose detection)

Install Python dependencies:
```bash
pip install opencv-python mediapipe
```
