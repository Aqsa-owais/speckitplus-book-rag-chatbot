# Simple Isaac ROS Perception Pipeline Example

This example demonstrates a basic perception pipeline using Isaac ROS components. The pipeline processes camera images to detect objects and estimate their 3D positions.

## Overview

The perception pipeline performs the following steps:
1. Subscribes to camera images and camera calibration information
2. Performs basic object detection using color-based detection
3. Estimates 3D positions of detected objects using camera parameters
4. Publishes detections and object positions in ROS message formats
5. Broadcasts TF transforms for detected objects

## Components

### Main Pipeline Node (`simple_perception_pipeline.py`)

The main pipeline node implements:
- Image subscription and processing
- Color-based object detection
- 3D position estimation
- Detection publishing in standard ROS formats
- TF broadcasting for detected objects

### Key Features

#### Object Detection
- Color-based detection for red, blue, and green objects
- Morphological operations for noise reduction
- Minimum area filtering for valid detections
- Confidence scoring based on object size

#### 3D Position Estimation
- Uses camera intrinsic parameters for 3D reconstruction
- Depth estimation based on object size in image
- Pixel-to-3D coordinate conversion

#### Message Formats
- Uses standard ROS message types (`sensor_msgs/Image`, `vision_msgs/Detection2DArray`)
- Publishes object positions as `geometry_msgs/PointStamped`
- Broadcasts transforms using TF2

## Running the Example

### Prerequisites

Before running the example, ensure you have:
- ROS 2 installed with vision and perception packages
- Isaac ROS perception packages (optional but recommended)
- Camera driver publishing calibrated images
- Proper camera calibration parameters available

### Installation

1. **Create a ROS 2 package for the example:**
```bash
mkdir -p ~/perception_pipeline_ws/src
cd ~/perception_pipeline_ws/src
ros2 pkg create --build-type ament_python perception_pipeline_examples
```

2. **Copy the example files:**
```bash
# Copy the Python script
cp simple_perception_pipeline.py ~/perception_pipeline_ws/src/perception_pipeline_examples/perception_pipeline_examples/

# Copy the launch file
cp launch_perception_pipeline.py ~/perception_pipeline_ws/src/perception_pipeline_examples/launch/
```

3. **Build the workspace:**
```bash
cd ~/perception_pipeline_ws
colcon build --packages-select perception_pipeline_examples
source install/setup.bash
```

### Execution

#### With Real Camera

1. **Start your camera driver:**
```bash
# Example for USB camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

2. **Run the perception pipeline:**
```bash
ros2 run perception_pipeline_examples simple_perception_pipeline
```

#### With Simulation

1. **Launch simulation environment:**
```bash
# Launch Gazebo with robot that has a camera
# This will publish images to /camera/image_raw
```

2. **Run the perception pipeline:**
```bash
ros2 launch perception_pipeline_examples launch_perception_pipeline.py
```

#### With Sample Images

1. **Play a bag file with camera data:**
```bash
ros2 bag play camera_data.bag
```

2. **Run the perception pipeline:**
```bash
ros2 run perception_pipeline_examples simple_perception_pipeline
```

## Topics and Messages

### Published Topics

- `/perception/detections` (`vision_msgs/Detection2DArray`): Object detections with bounding boxes and classifications
- `/perception/object_poses` (`geometry_msgs/PointStamped`): 3D positions of detected objects
- TF transforms: Transforms for detected objects

### Subscribed Topics

- `/camera/image_raw` (`sensor_msgs/Image`): Raw camera images
- `/camera/camera_info` (`sensor_msgs/CameraInfo`): Camera calibration parameters

## Parameters

The pipeline has several configurable parameters:

- `min_object_area`: Minimum area for valid object detection (default: 100 pixels)
- `object_colors`: Dictionary of colors to detect with HSV ranges
- `use_sim_time`: Whether to use simulation time (default: false)

## Integration with Isaac ROS

This example demonstrates integration with Isaac ROS concepts:

### 1. Standard Message Formats
The pipeline uses standard ROS message formats that are compatible with Isaac ROS components:
- `vision_msgs/Detection2DArray` for object detections
- `sensor_msgs/Image` for image data
- `geometry_msgs/PointStamped` for 3D positions

### 2. TF Integration
The pipeline broadcasts TF transforms for detected objects, enabling integration with Isaac ROS's spatial reasoning capabilities.

### 3. Modularity
The pipeline is designed to be modular and can be easily extended with more sophisticated Isaac ROS perception components.

## Extending the Example

### Adding More Detection Types

To add more object detection capabilities:

1. **Modify the detection function:**
```python
def detect_objects(self, image):
    # Add your detection method here
    aruco_detections = self.detect_aruco_markers(image)
    yolov5_detections = self.detect_with_yolo(image)
    # Combine all detections
```

### Improving Depth Estimation

Replace the simplified depth estimation with more accurate methods:

1. **Stereo Vision:** Use stereo camera setup for accurate depth
2. **LiDAR Fusion:** Combine camera and LiDAR data
3. **Learned Depth:** Use neural networks for monocular depth estimation

### Adding Isaac ROS Components

Replace simple components with Isaac ROS equivalents:

```python
# Instead of simple color detection, use Isaac ROS detection
from isaac_ros_detectnet import DetectNetNode

detectnet_node = DetectNetNode(
    input_topic='/camera/image_rect_color',
    output_topic='/detectnet/detections'
)
```

## Performance Considerations

### Real-time Performance

The pipeline is designed for real-time performance:
- Efficient color-based detection
- Optimized image processing operations
- Asynchronous message handling

### GPU Acceleration

For improved performance, consider using GPU-accelerated components:
- Isaac ROS provides GPU-accelerated detection networks
- OpenCV CUDA functions for image processing
- TensorRT for neural network inference

## Troubleshooting

### Common Issues

1. **No Detections**
   - Check camera calibration parameters
   - Verify lighting conditions
   - Adjust color ranges for your environment

2. **Incorrect 3D Positions**
   - Verify camera intrinsics are correct
   - Check camera mounting position
   - Validate depth estimation assumptions

3. **Performance Issues**
   - Reduce image resolution
   - Lower processing frequency
   - Use GPU acceleration

### Debugging

Enable debug output to troubleshoot issues:
```bash
# Run with debug logging
ros2 run perception_pipeline_examples simple_perception_pipeline --ros-args -p __log_level:=DEBUG
```

## Related Isaac ROS Packages

This example complements several Isaac ROS packages:
- `isaac_ros_detectnet`: Object detection with neural networks
- `isaac_ros_visual_slam`: Visual SLAM capabilities
- `isaac_ros_point_cloud_utils`: Point cloud processing
- `isaac_ros_apriltag`: AprilTag detection and pose estimation

## License

This example is provided under the Apache 2.0 license. See the license file for details.