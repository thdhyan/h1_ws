#!/usr/bin/env python3
"""
MediaPipe Pose Detection Node
Subscribes to a camera stream and runs MediaPipe pose detection
Publishes pose landmarks and annotated image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from h1_msgs.msg import PoseDetection, PoseLandmark
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False


class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        
        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().error('MediaPipe is not installed. Please install it with: pip install mediapipe')
            return
        
        # Declare parameters
        self.declare_parameter('input_topic', 'camera/image_raw')
        self.declare_parameter('output_pose_topic', 'pose/detection')
        self.declare_parameter('output_image_topic', 'pose/image_annotated')
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)
        self.declare_parameter('model_complexity', 1)  # 0, 1, or 2
        self.declare_parameter('enable_segmentation', False)
        self.declare_parameter('smooth_landmarks', True)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.output_image_topic = self.get_parameter('output_image_topic').value
        min_detection_confidence = self.get_parameter('min_detection_confidence').value
        min_tracking_confidence = self.get_parameter('min_tracking_confidence').value
        model_complexity = self.get_parameter('model_complexity').value
        enable_segmentation = self.get_parameter('enable_segmentation').value
        smooth_landmarks = self.get_parameter('smooth_landmarks').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
            model_complexity=model_complexity,
            enable_segmentation=enable_segmentation,
            smooth_landmarks=smooth_landmarks
        )
        
        # Create subscriber
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Create publishers
        self.pose_pub = self.create_publisher(PoseDetection, self.output_pose_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)
        
        self.get_logger().info(f'Pose detection node started')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing pose to: {self.output_pose_topic}')
        self.get_logger().info(f'Publishing annotated image to: {self.output_image_topic}')
    
    def image_callback(self, msg):
        """Process incoming images and detect poses"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to RGB for MediaPipe
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = self.pose.process(image_rgb)
            
            # Create annotated image
            annotated_image = cv_image.copy()
            
            # Create pose detection message
            pose_msg = PoseDetection()
            pose_msg.header = msg.header
            
            if results.pose_landmarks:
                # Draw landmarks on image
                self.mp_drawing.draw_landmarks(
                    annotated_image,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
                )
                
                # Convert landmarks to message
                pose_msg.detected = True
                pose_msg.confidence = 1.0  # MediaPipe doesn't provide overall confidence
                
                for landmark in results.pose_landmarks.landmark:
                    landmark_msg = PoseLandmark()
                    landmark_msg.x = landmark.x
                    landmark_msg.y = landmark.y
                    landmark_msg.z = landmark.z
                    landmark_msg.visibility = landmark.visibility
                    landmark_msg.presence = getattr(landmark, 'presence', 1.0)
                    pose_msg.landmarks.append(landmark_msg)
            else:
                pose_msg.detected = False
                pose_msg.confidence = 0.0
                
                # Add "No Pose Detected" text
                cv2.putText(annotated_image, 'No Pose Detected', (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Publish pose detection
            self.pose_pub.publish(pose_msg)
            
            # Publish annotated image
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing annotated image: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'Error in pose detection: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources"""
        if MEDIAPIPE_AVAILABLE and hasattr(self, 'pose'):
            self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
