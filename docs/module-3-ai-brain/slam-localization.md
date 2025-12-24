---
sidebar_position: 8
---

# Visual SLAM and Localization

## Introduction

Simultaneous Localization and Mapping (SLAM) is a fundamental capability for autonomous robots, enabling them to build a map of an unknown environment while simultaneously determining their position within that map. Visual SLAM specifically uses camera sensors to extract features and landmarks for mapping and localization. This guide covers Visual SLAM concepts, algorithms, and implementation using Isaac ROS and related tools.

## SLAM Fundamentals

### The SLAM Problem

SLAM addresses the "chicken and egg" problem in robotics: to build a map, you need to know where you are, but to know where you are, you need a map. The SLAM solution jointly estimates:

1. **Robot Trajectory**: The sequence of robot poses over time
2. **Map**: The locations of landmarks or features in the environment
3. **Current State**: The robot's current pose relative to the map

### SLAM Mathematical Framework

The SLAM problem can be formulated as a probabilistic estimation problem:

```
P(x_t, m | z_1:t, u_1:t)
```

Where:
- `x_t` is the robot trajectory up to time t
- `m` is the map
- `z_1:t` is the sequence of observations
- `u_1:t` is the sequence of control inputs

## Visual SLAM Approaches

### Feature-Based Visual SLAM

Feature-based methods extract and track distinctive features (points, lines, corners) in the environment:

```python
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class FeatureBasedVisualSLAM:
    def __init__(self):
        # Feature detector and descriptor
        self.detector = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher()

        # Camera parameters
        self.K = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Pose tracking
        self.current_pose = np.eye(4)
        self.keyframes = []
        self.map_points = []

    def process_frame(self, image):
        """Process a new frame for SLAM"""
        # Detect features
        keypoints, descriptors = self.detector.detectAndCompute(image, None)

        if len(keypoints) < 100:  # Need sufficient features
            return self.current_pose

        # Track features from previous frame
        if hasattr(self, 'prev_keypoints') and self.prev_descriptors is not None:
            matches = self.matcher.knnMatch(
                descriptors, self.prev_descriptors, k=2
            )

            # Apply Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)

            if len(good_matches) >= 10:  # Need sufficient matches
                # Extract matched points
                prev_pts = np.float32([
                    self.prev_keypoints[m.trainIdx].pt for m in good_matches
                ]).reshape(-1, 1, 2)
                curr_pts = np.float32([
                    keypoints[m.queryIdx].pt for m in good_matches
                ]).reshape(-1, 1, 2)

                # Estimate essential matrix
                E, mask = cv2.findEssentialMat(
                    curr_pts, prev_pts, self.K,
                    method=cv2.RANSAC, threshold=1.0
                )

                if E is not None:
                    # Recover pose
                    _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, self.K)

                    # Update current pose
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t.flatten()
                    self.current_pose = self.current_pose @ np.linalg.inv(T)

        # Store current frame for next iteration
        self.prev_keypoints = keypoints
        self.prev_descriptors = descriptors

        return self.current_pose
```

### Direct Visual SLAM

Direct methods use pixel intensities directly rather than extracting features:

```python
class DirectVisualSLAM:
    def __init__(self):
        self.reference_frame = None
        self.depth_map = None
        self.current_pose = np.eye(4)

    def estimate_motion_direct(self, current_frame, reference_frame, K):
        """Estimate motion using direct method"""
        # Compute image gradients
        grad_x = cv2.Sobel(reference_frame, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(reference_frame, cv2.CV_64F, 0, 1, ksize=3)

        # Initialize pose increment
        xi = np.zeros(6)  # [rx, ry, rz, tx, ty, tz]

        # Iterative optimization (simplified)
        for iteration in range(10):
            # Warp current frame based on current estimate
            warped_frame = self.warp_frame(
                current_frame, reference_frame, self.depth_map,
                self.current_pose, K, xi
            )

            # Compute photometric error
            error = reference_frame.astype(float) - warped_frame.astype(float)

            # Compute Jacobian (simplified)
            # In practice, this would involve more complex derivations
            jacobian = self.compute_jacobian(grad_x, grad_y, self.depth_map)

            # Update pose estimate
            delta_xi = np.linalg.lstsq(jacobian, error.flatten(), rcond=None)[0]
            xi += delta_xi

        # Update pose
        self.current_pose = self.pose_vector_to_matrix(xi) @ self.current_pose

        return self.current_pose

    def warp_frame(self, current_frame, reference_frame, depth_map, pose, K, xi):
        """Warp current frame to reference frame"""
        # Implementation details for frame warping
        # This is a simplified placeholder
        return current_frame

    def compute_jacobian(self, grad_x, grad_y, depth_map):
        """Compute Jacobian matrix for direct method"""
        # Implementation details
        return np.eye(6)
```

### Semi-Direct Methods

Semi-direct methods combine feature tracking with direct alignment:

```python
class SemiDirectSLAM:
    def __init__(self):
        # Feature tracking for robust matching
        self.feature_tracker = FeatureBasedVisualSLAM()

        # Direct alignment for precision
        self.direct_aligner = DirectVisualSLAM()

        # Map management
        self.map = Map()
        self.local_window = []

    def process_frame(self, image):
        """Process frame using semi-direct approach"""
        # Step 1: Track features for robust initialization
        tracked_pose = self.feature_tracker.process_frame(image)

        # Step 2: Refine using direct alignment
        refined_pose = self.direct_aligner.estimate_motion_direct(
            image, self.get_reference_frame(), self.feature_tracker.K
        )

        # Step 3: Update map and local window
        self.update_map(image, refined_pose)

        return refined_pose

    def update_map(self, image, pose):
        """Update map with new observations"""
        # Add keyframe if significant motion detected
        if self.is_keyframe_needed(pose):
            self.add_keyframe(image, pose)

        # Optimize local window
        self.optimize_local_window()

    def is_keyframe_needed(self, pose):
        """Determine if current frame should be a keyframe"""
        # Check for sufficient translation/rotation
        if len(self.local_window) == 0:
            return True

        last_pose = self.local_window[-1].pose
        delta_t = np.linalg.norm(pose[:3, 3] - last_pose[:3, 3])
        delta_r = R.from_matrix(pose[:3, :3] @ last_pose[:3, :3].T).as_rotvec()

        return delta_t > 0.1 or np.linalg.norm(delta_r) > 0.1
```

## Isaac ROS Visual SLAM Implementation

### Isaac ROS Stereo Visual Odometry

Isaac ROS provides optimized stereo visual odometry nodes:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np

class IsaacStereoOdometryNode(Node):
    def __init__(self):
        super().__init__('isaac_stereo_odometry')

        # Subscribe to stereo images
        self.left_sub = self.create_subscription(
            Image,
            '/stereo/left/image_rect',
            self.left_callback,
            10
        )

        self.right_sub = self.create_subscription(
            Image,
            '/stereo/right/image_rect',
            self.right_callback,
            10
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_pose', 10)

        self.bridge = CvBridge()

        # Initialize stereo odometry
        self.left_image = None
        self.right_image = None
        self.initialized = False
        self.current_pose = np.eye(4)
        self.prev_pose = np.eye(4)

        # Camera parameters (should be loaded from calibration)
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Stereo baseline (distance between cameras)
        self.baseline = 0.1  # 10 cm

    def left_callback(self, msg):
        """Process left camera image"""
        if not self.initialized:
            self.initialize_stereo_odometry(msg)
            return

        self.process_stereo_odometry(msg, self.right_image)

    def right_callback(self, msg):
        """Process right camera image"""
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def initialize_stereo_odometry(self, left_msg):
        """Initialize stereo odometry with first frame"""
        self.left_image = self.bridge.imgmsg_to_cv2(left_msg, "mono8")
        self.initialized = True
        self.get_logger().info("Stereo odometry initialized")

    def process_stereo_odometry(self, left_msg, right_img):
        """Process stereo images for odometry"""
        if right_img is None:
            return

        left_img = self.bridge.imgmsg_to_cv2(left_msg, "mono8")

        try:
            # Extract features from left image
            kp1, desc1 = self.extract_features(left_img)

            # Extract features from previous left image
            kp2, desc2 = self.extract_features(self.left_image)

            # Match features
            matches = self.match_features(desc1, desc2)

            if len(matches) >= 10:
                # Estimate motion
                motion = self.estimate_motion(kp1, kp2, matches)

                # Update pose
                self.prev_pose = self.current_pose.copy()
                self.current_pose = self.current_pose @ motion

                # Publish odometry
                self.publish_odometry(left_msg.header)

            # Update previous image
            self.left_image = left_img

        except Exception as e:
            self.get_logger().error(f'Error in stereo odometry: {e}')

    def extract_features(self, image):
        """Extract SIFT features from image"""
        detector = cv2.SIFT_create(nfeatures=1000)
        kp, desc = detector.detectAndCompute(image, None)
        return kp, desc

    def match_features(self, desc1, desc2):
        """Match features between two descriptors"""
        if desc1 is None or desc2 is None:
            return []

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(desc1, desc2, k=2)

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)

        return good_matches

    def estimate_motion(self, kp1, kp2, matches):
        """Estimate 3D motion from matched features"""
        if len(matches) < 10:
            return np.eye(4)

        # Get matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Estimate fundamental matrix
        F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.RANSAC, 4, 0.999)
        if F is None:
            return np.eye(4)

        # Estimate essential matrix
        E = self.camera_matrix.T @ F @ self.camera_matrix

        # Recover pose
        _, R, t, _ = cv2.recoverPose(E, pts1, pts2, self.camera_matrix)

        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()

        return T

    def publish_odometry(self, header):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "camera"

        # Convert pose to ROS format
        pose = self.current_pose
        odom_msg.pose.pose.position.x = pose[0, 3]
        odom_msg.pose.pose.position.y = pose[1, 3]
        odom_msg.pose.pose.position.z = pose[2, 3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(pose[:3, :3])
        quat = r.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocity would be computed from pose differences
        # For now, set to zero
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

        # Also publish as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "map"
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)
```

### Isaac ROS Loop Closure and Global Optimization

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.distance import cdist

class IsaacLoopClosureNode(Node):
    def __init__(self):
        super().__init__('isaac_loop_closure')

        # Subscribe to pose estimates
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # or visual odometry pose
            self.pose_callback,
            10
        )

        # Publish corrected poses
        self.corrected_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/corrected_pose',
            10
        )

        # Loop closure detection
        self.pose_history = []
        self.loop_threshold = 2.0  # meters
        self.min_loop_features = 50

        # Graph optimization parameters
        self.optimization_queue = []
        self.optimization_threshold = 10  # poses before optimization

    def pose_callback(self, msg):
        """Process incoming pose estimates"""
        # Store pose in history
        pose_data = {
            'timestamp': msg.header.stamp,
            'pose': msg.pose.pose,
            'position': np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]),
            'features': self.extract_local_features(msg)  # Simplified
        }

        self.pose_history.append(pose_data)

        # Check for potential loop closures
        loop_candidates = self.find_loop_candidates(pose_data)

        if loop_candidates:
            # Verify loop closure
            confirmed_loops = self.verify_loop_closure(pose_data, loop_candidates)

            if confirmed_loops:
                # Optimize graph
                self.optimize_graph(confirmed_loops)

    def find_loop_candidates(self, current_pose_data):
        """Find potential loop closure candidates"""
        candidates = []

        # Simple distance-based search
        current_pos = current_pose_data['position']

        for i, past_pose in enumerate(self.pose_history[:-10]):  # Skip recent poses
            distance = np.linalg.norm(current_pos - past_pose['position'])

            if distance < self.loop_threshold:
                candidates.append(i)

        return candidates

    def verify_loop_closure(self, current_pose_data, candidates):
        """Verify potential loop closures using feature matching"""
        confirmed = []

        for idx in candidates:
            past_pose = self.pose_history[idx]

            # Match features between current and past pose
            if self.match_features(current_pose_data['features'], past_pose['features']):
                confirmed.append(idx)

        return confirmed

    def match_features(self, features1, features2):
        """Match features between two sets (simplified)"""
        # In practice, this would use more sophisticated feature matching
        # For now, just check if we have enough features
        return (len(features1) > self.min_loop_features and
                len(features2) > self.min_loop_features)

    def optimize_graph(self, loop_constraints):
        """Optimize pose graph with loop closure constraints"""
        # This is a simplified placeholder
        # In practice, would use libraries like g2o or Ceres Solver

        # For demonstration, just adjust poses based on constraints
        for loop_idx in loop_constraints:
            # Adjust poses to minimize drift
            self.adjust_poses_for_loop_closure(loop_idx)

    def extract_local_features(self, pose_msg):
        """Extract local features for loop closure detection"""
        # In practice, this would extract visual features from camera
        # For now, return a simple descriptor
        return np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.orientation.w  # Simplified
        ])
```

## Localization Techniques

### Monte Carlo Localization (Particle Filter)

```python
import numpy as np
from scipy.stats import norm
from geometry_msgs.msg import PoseArray, Pose

class ParticleFilterLocalization:
    def __init__(self, map_resolution=0.05, map_size=(200, 200)):
        self.map_resolution = map_resolution
        self.map_size = map_size
        self.map = np.zeros(map_size)  # Occupancy grid map

        # Particle filter parameters
        self.num_particles = 1000
        self.particles = np.zeros((self.num_particles, 3))  # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Motion model noise
        self.motion_noise = [0.1, 0.1, 0.05]  # x, y, theta

        # Sensor model parameters
        self.sensor_std = 0.1

    def initialize_particles_uniform(self):
        """Initialize particles uniformly across the map"""
        for i in range(self.num_particles):
            x = np.random.uniform(0, self.map_size[0] * self.map_resolution)
            y = np.random.uniform(0, self.map_size[1] * self.map_resolution)
            theta = np.random.uniform(-np.pi, np.pi)
            self.particles[i] = [x, y, theta]

    def predict(self, control):
        """Predict particle motion based on control input"""
        # Add noise to control input
        noisy_control = [
            control[0] + np.random.normal(0, self.motion_noise[0]),
            control[1] + np.random.normal(0, self.motion_noise[1]),
            control[2] + np.random.normal(0, self.motion_noise[2])
        ]

        # Update particle poses
        for i in range(self.num_particles):
            self.particles[i, 0] += noisy_control[0] * np.cos(self.particles[i, 2])
            self.particles[i, 1] += noisy_control[0] * np.sin(self.particles[i, 2])
            self.particles[i, 2] += noisy_control[2]

            # Normalize angle
            self.particles[i, 2] = self.normalize_angle(self.particles[i, 2])

    def update(self, observations):
        """Update particle weights based on sensor observations"""
        for i in range(self.num_particles):
            weight = 1.0

            for obs in observations:
                # Predict what this particle expects to see
                expected_obs = self.predict_observation(self.particles[i])

                # Calculate likelihood of actual observation
                likelihood = self.calculate_likelihood(obs, expected_obs)
                weight *= likelihood

            self.weights[i] = weight

        # Normalize weights
        self.weights += 1.e-300  # Avoid numerical issues
        self.weights /= np.sum(self.weights)

        # Resample if effective sample size is too low
        if self.effective_sample_size() < self.num_particles / 2:
            self.resample()

    def predict_observation(self, particle_pose):
        """Predict sensor observations for a given particle pose"""
        # This would typically involve ray-casting or other sensor models
        # Simplified for demonstration
        return np.array([particle_pose[0], particle_pose[1]])

    def calculate_likelihood(self, actual, expected):
        """Calculate likelihood of observation"""
        diff = actual - expected
        distance = np.linalg.norm(diff)
        return norm.pdf(distance, 0, self.sensor_std)

    def effective_sample_size(self):
        """Calculate effective sample size"""
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self):
        """Resample particles based on weights"""
        indices = np.random.choice(
            self.num_particles,
            size=self.num_particles,
            p=self.weights
        )

        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate_pose(self):
        """Estimate robot pose from particles"""
        # Weighted average of particles
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)

        # For angle, need to handle circular statistics
        cos_sum = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        sin_sum = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        theta = np.arctan2(sin_sum, cos_sum)

        return np.array([x, y, theta])

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

### Extended Kalman Filter (EKF) Localization

```python
class EKFLocalization:
    def __init__(self):
        # State: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 0.1

        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.01, 0.1, 0.1, 0.01])

        # Measurement noise (for different sensor types)
        self.R_landmark = np.diag([0.1, 0.1])  # range, bearing
        self.R_odometry = np.diag([0.05, 0.05, 0.01])  # x, y, theta

    def predict(self, control, dt):
        """Predict state and covariance"""
        # Motion model: constant velocity
        F = self.compute_jacobian_motion()

        # Update state
        self.state[0] += self.state[3] * dt  # x = x + vx * dt
        self.state[1] += self.state[4] * dt  # y = y + vy * dt
        self.state[2] += self.state[5] * dt  # theta = theta + omega * dt

        # Update covariance
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update_landmark(self, landmark_pos, observed_range, observed_bearing):
        """Update with landmark observation"""
        # Predicted measurement
        dx = landmark_pos[0] - self.state[0]
        dy = landmark_pos[1] - self.state[1]
        predicted_range = np.sqrt(dx**2 + dy**2)
        predicted_bearing = np.arctan2(dy, dx) - self.state[2]

        # Measurement model Jacobian
        H = self.compute_jacobian_landmark(landmark_pos)

        # Innovation
        innovation = np.array([
            observed_range - predicted_range,
            self.normalize_angle(observed_bearing - predicted_bearing)
        ])

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_landmark

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state += K @ innovation
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance

    def compute_jacobian_motion(self):
        """Compute motion model Jacobian"""
        dt = 0.1  # time step
        F = np.eye(6)
        F[0, 3] = dt  # dx/dvx
        F[1, 4] = dt  # dy/dvy
        F[2, 5] = dt  # dtheta/domega
        return F

    def compute_jacobian_landmark(self, landmark_pos):
        """Compute landmark measurement Jacobian"""
        dx = landmark_pos[0] - self.state[0]
        dy = landmark_pos[1] - self.state[1]
        q = dx**2 + dy**2
        sqrt_q = np.sqrt(q)

        H = np.zeros((2, 6))
        H[0, 0] = -dx / sqrt_q  # dr/dx
        H[0, 1] = -dy / sqrt_q  # dr/dy
        H[1, 0] = dy / q        # db/dx
        H[1, 1] = -dx / q       # db/dy
        H[1, 2] = -1            # db/dtheta

        return H

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

## Mapping Techniques

### Occupancy Grid Mapping

```python
class OccupancyGridMapper:
    def __init__(self, resolution=0.05, width=20, height=20):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((int(height/resolution), int(width/resolution)))

        # Log-odds representation
        self.log_odds = np.zeros_like(self.grid)
        self.occupied_threshold = 0.5

        # Sensor parameters
        self.max_range = 10.0
        self.angle_increment = 0.01

    def update_map(self, scan_data, robot_pose):
        """Update occupancy grid with laser scan data"""
        # Convert scan to world coordinates
        world_points = self.scan_to_world(scan_data, robot_pose)

        # Get free space points (endpoints of valid rays)
        endpoints = self.get_valid_endpoints(scan_data, robot_pose)

        # Update grid with hits and misses
        for point in world_points:
            grid_x, grid_y = self.world_to_grid(point)
            if self.is_valid_grid_cell(grid_x, grid_y):
                self.log_odds[grid_y, grid_x] += self.log_odds_occupied

        for endpoint in endpoints:
            grid_x, grid_y = self.world_to_grid(endpoint)
            if self.is_valid_grid_cell(grid_x, grid_y):
                self.log_odds[grid_y, grid_x] += self.log_odds_free

        # Convert back to probability
        self.grid = self.log_odds_to_probability(self.log_odds)

    def scan_to_world(self, scan_data, robot_pose):
        """Convert laser scan to world coordinates"""
        points = []
        robot_x, robot_y, robot_theta = robot_pose

        for i, range_reading in enumerate(scan_data.ranges):
            if 0 < range_reading < self.max_range:
                angle = scan_data.angle_min + i * scan_data.angle_increment
                world_x = robot_x + range_reading * np.cos(robot_theta + angle)
                world_y = robot_y + range_reading * np.sin(robot_theta + angle)
                points.append([world_x, world_y])

        return points

    def get_valid_endpoints(self, scan_data, robot_pose):
        """Get endpoints of valid laser rays"""
        endpoints = []
        robot_x, robot_y, robot_theta = robot_pose

        for i, range_reading in enumerate(scan_data.ranges):
            if 0 < range_reading < self.max_range:
                angle = scan_data.angle_min + i * scan_data.angle_increment
                world_x = robot_x + range_reading * np.cos(robot_theta + angle)
                world_y = robot_y + range_reading * np.sin(robot_theta + angle)
                endpoints.append([world_x, world_y])

        return endpoints

    def world_to_grid(self, world_point):
        """Convert world coordinates to grid indices"""
        grid_x = int((world_point[0] + self.width/2) / self.resolution)
        grid_y = int((world_point[1] + self.height/2) / self.resolution)
        return grid_x, grid_y

    def is_valid_grid_cell(self, x, y):
        """Check if grid coordinates are valid"""
        return (0 <= x < self.grid.shape[1]) and (0 <= y < self.grid.shape[0])

    def log_odds_to_probability(self, log_odds):
        """Convert log-odds to probability"""
        prob = 1 - 1 / (1 + np.exp(log_odds))
        return prob
```

## Performance Optimization

### Multi-Threaded SLAM Architecture

```python
import threading
import queue
import time

class MultiThreadedSLAM:
    def __init__(self):
        # Queues for inter-thread communication
        self.image_queue = queue.Queue(maxsize=5)
        self.feature_queue = queue.Queue(maxsize=10)
        self.optimization_queue = queue.Queue(maxsize=5)

        # Threading locks
        self.map_lock = threading.Lock()

        # Worker threads
        self.feature_thread = threading.Thread(target=self.feature_extraction_worker)
        self.optimization_thread = threading.Thread(target=self.optimization_worker)
        self.mapping_thread = threading.Thread(target=self.mapping_worker)

        # Start threads
        self.feature_thread.start()
        self.optimization_thread.start()
        self.mapping_thread.start()

    def process_image(self, image):
        """Main entry point for image processing"""
        try:
            self.image_queue.put_nowait(image)
        except queue.Full:
            # Drop frame if queue is full
            pass

    def feature_extraction_worker(self):
        """Extract features from images in background"""
        while True:
            try:
                image = self.image_queue.get(timeout=1.0)

                # Extract features
                features = self.extract_features(image)

                # Add to feature queue
                self.feature_queue.put((image, features))

            except queue.Empty:
                continue

    def optimization_worker(self):
        """Run graph optimization in background"""
        while True:
            try:
                optimization_data = self.optimization_queue.get(timeout=1.0)

                # Perform optimization
                optimized_poses = self.optimize_graph(optimization_data)

                # Update global map
                with self.map_lock:
                    self.update_global_map(optimized_poses)

            except queue.Empty:
                continue

    def mapping_worker(self):
        """Build map from features and poses"""
        while True:
            try:
                image, features = self.feature_queue.get(timeout=1.0)

                # Estimate pose and update map
                pose = self.estimate_pose(features)
                self.update_local_map(image, pose)

                # Add to optimization queue if needed
                if self.should_optimize():
                    optimization_data = self.prepare_optimization_data()
                    self.optimization_queue.put(optimization_data)

            except queue.Empty:
                continue
```

## Isaac ROS Integration

### Launch Configuration for Visual SLAM

```xml
<!-- Example launch file for Isaac ROS Visual SLAM -->
<launch>
  <!-- Stereo camera drivers -->
  <node pkg="isaac_ros_stereo_image_proc" exec="isaac_ros_stereo_rectify" name="stereo_rectify">
    <param name="left_topic" value="/left/image_raw"/>
    <param name="right_topic" value="/right/image_raw"/>
    <param name="left_camera_info_url" value="file://$(find-pkg-share my_robot_description)/config/left_camera.yaml"/>
    <param name="right_camera_info_url" value="file://$(find-pkg-share my_robot_description)/config/right_camera.yaml"/>
  </node>

  <!-- Visual odometry -->
  <node pkg="isaac_ros_visual_odometry" exec="isaac_ros_visual_odometry" name="visual_odometry">
    <param name="left_rect_topic" value="/left/image_rect"/>
    <param name="right_rect_topic" value="/right/image_rect"/>
    <param name="max_num_features" value="1000"/>
    <param name="min_num_features" value="100"/>
  </node>

  <!-- Loop closure detection -->
  <node pkg="isaac_ros_loop_closure" exec="isaac_ros_loop_closure" name="loop_closure">
    <param name="pose_topic" value="/visual_odom"/>
    <param name="loop_threshold" value="2.0"/>
  </node>

  <!-- Map building -->
  <node pkg="isaac_ros_map_building" exec="isaac_ros_map_builder" name="map_builder">
    <param name="submap_resolution" value="0.05"/>
    <param name="map_publish_period" value="1.0"/>
  </node>

  <!-- Localization -->
  <node pkg="isaac_ros_localization" exec="isaac_ros_localization" name="localization">
    <param name="initial_pose_topic" value="/initialpose"/>
    <param name="map_topic" value="/map"/>
  </node>
</launch>
```

## Troubleshooting Common Issues

### 1. Drift in Visual Odometry
- **Problem**: Accumulating position error over time
- **Solution**: Implement loop closure, use sensor fusion with IMU, optimize feature tracking

### 2. Feature Loss in Textureless Environments
- **Problem**: Insufficient features for tracking
- **Solution**: Use direct methods, combine with other sensors, implement featureless SLAM

### 3. Scale Ambiguity in Monocular SLAM
- **Problem**: Cannot determine absolute scale from single camera
- **Solution**: Use stereo cameras, add IMU data, use known object sizes

### 4. Real-time Performance Issues
- **Problem**: SLAM pipeline too slow for real-time operation
- **Solution**: Optimize algorithms, reduce feature count, use GPU acceleration

## Summary

Visual SLAM and localization are critical capabilities for autonomous robots, enabling them to navigate and operate in unknown environments. This guide covered the fundamental concepts, algorithms, and implementation approaches for visual SLAM, including feature-based methods, direct methods, and Isaac ROS integration. Proper implementation of SLAM systems requires careful attention to sensor calibration, algorithm optimization, and real-time performance considerations. The combination of visual perception and robust localization forms the foundation for intelligent robot navigation and mapping systems.