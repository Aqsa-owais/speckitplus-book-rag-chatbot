#!/usr/bin/env python3
"""
Simple Isaac ROS Perception Pipeline Example

This example demonstrates a basic perception pipeline using Isaac ROS components.
The pipeline processes camera images to detect objects and estimate their 3D positions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class SimplePerceptionPipeline(Node):
    """
    A simple perception pipeline that:
    1. Subscribes to camera images and camera info
    2. Performs basic object detection (using color-based detection for demonstration)
    3. Estimates 3D positions of detected objects
    4. Publishes detections and object positions
    """

    def __init__(self):
        super().__init__('simple_perception_pipeline')

        # Create CV bridge for image conversion
        self.bridge = CvBridge()

        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.object_poses_pub = self.create_publisher(PointStamped, '/perception/object_poses', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # TF broadcaster for object transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera parameters (will be populated from camera info)
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False

        # Processing parameters
        self.min_object_area = 100  # Minimum area for valid detection
        self.object_colors = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255])
        }

        self.get_logger().info("Simple perception pipeline initialized")

    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        if not self.camera_info_received:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera calibration parameters received")

    def image_callback(self, msg):
        """Process incoming camera image"""
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for camera calibration parameters...")
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Process detections
            self.process_detections(detections, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detect_objects(self, image):
        """Detect objects in the image using color-based detection"""
        detections = []

        # Convert BGR to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for color_name, (lower, upper) in self.object_colors.items():
            # Create mask for the current color
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)

                # Only consider contours with sufficient area
                if area > self.min_object_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate center
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Calculate 3D position (simplified)
                    depth = self.estimate_depth_from_size(w, h)
                    position_3d = self.pixel_to_3d((center_x, center_y), depth)

                    detection = {
                        'color': color_name,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'area': area,
                        'position_3d': position_3d,
                        'confidence': min(area / 1000.0, 1.0)  # Normalize confidence
                    }

                    detections.append(detection)

        return detections

    def estimate_depth_from_size(self, width_pixels, height_pixels):
        """
        Estimate depth based on object size in pixels (simplified method)
        In practice, you would use stereo vision or depth sensors
        """
        # This is a simplified depth estimation
        # In real applications, use proper depth sensing (stereo, LiDAR, etc.)
        avg_dimension = (width_pixels + height_pixels) / 2

        # Calibrated relationship: larger objects are closer
        # This is just an example - real calibration needed
        if avg_dimension > 100:
            return 0.5  # Close
        elif avg_dimension > 50:
            return 1.0  # Medium distance
        else:
            return 2.0  # Far

    def pixel_to_3d(self, pixel_coords, depth):
        """Convert 2D pixel coordinates to 3D world coordinates"""
        if self.camera_matrix is None:
            return (0, 0, depth)

        u, v = pixel_coords
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # Convert to normalized coordinates
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy

        # Convert to 3D
        x_3d = x_norm * depth
        y_3d = y_norm * depth
        z_3d = depth

        return (x_3d, y_3d, z_3d)

    def process_detections(self, detections, header):
        """Process detections and publish results"""
        # Create detection array message
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            # Create detection message
            detection_msg = self.create_detection_message(detection, header)
            detection_array.detections.append(detection_msg)

            # Publish object pose
            self.publish_object_pose(detection['position_3d'], header)

            # Broadcast transform
            self.broadcast_object_transform(detection, header)

        # Publish all detections
        self.detections_pub.publish(detection_array)

        # Log detection summary
        if detections:
            self.get_logger().info(f"Detected {len(detections)} objects: {[d['color'] for d in detections]}")

    def create_detection_message(self, detection, header):
        """Create Detection2D message from detection data"""
        detection_msg = Detection2D()
        detection_msg.header = header

        # Set bounding box
        bbox = detection['bbox']
        detection_msg.bbox.center.x = bbox[0] + bbox[2] / 2
        detection_msg.bbox.center.y = bbox[1] + bbox[3] / 2
        detection_msg.bbox.size_x = bbox[2]
        detection_msg.bbox.size_y = bbox[3]

        # Set classification result
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = detection['color']
        hypothesis.hypothesis.score = detection['confidence']

        detection_msg.results.append(hypothesis)

        return detection_msg

    def publish_object_pose(self, position_3d, header):
        """Publish 3D position of detected object"""
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.point.x = position_3d[0]
        point_msg.point.y = position_3d[1]
        point_msg.point.z = position_3d[2]

        self.object_poses_pub.publish(point_msg)

    def broadcast_object_transform(self, detection, header):
        """Broadcast TF transform for detected object"""
        t = TransformStamped()

        # Set transform header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = header.frame_id
        t.child_frame_id = f"detected_{detection['color']}_object"

        # Set transform
        pos = detection['position_3d']
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        # No rotation - identity quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def visualize_detections(self, image, detections):
        """Draw detections on image for visualization (optional)"""
        vis_image = image.copy()

        for detection in detections:
            # Draw bounding box
            bbox = detection['bbox']
            cv2.rectangle(vis_image, (bbox[0], bbox[1]),
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]),
                         (0, 255, 0), 2)

            # Draw center
            center = detection['center']
            cv2.circle(vis_image, center, 5, (0, 0, 255), -1)

            # Draw label
            label = f"{detection['color']}: {detection['confidence']:.2f}"
            cv2.putText(vis_image, label, (bbox[0], bbox[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return vis_image


def main(args=None):
    rclpy.init(args=args)

    perception_pipeline = SimplePerceptionPipeline()

    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()