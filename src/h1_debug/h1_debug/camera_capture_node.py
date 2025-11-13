#!/usr/bin/env python3
"""
Camera Capture Node using OpenCV
Captures video from a specified camera ID and publishes image and camera_info topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('output_topic', 'camera/image_raw')
        self.declare_parameter('visualization_topic', 'visualization/frame_1')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('namespace', '')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # Get parameters
        camera_id = self.get_parameter('camera_id').value
        output_topic = self.get_parameter('output_topic').value
        visualization_topic = self.get_parameter('visualization_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        namespace = self.get_parameter('namespace').value
        self.frame_id = self.get_parameter('frame_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        # Add namespace to topics if provided
        if namespace:
            output_topic = f'{namespace}/{output_topic}'
            visualization_topic = f'{namespace}/{visualization_topic}'
            camera_info_topic = f'{namespace}/{camera_info_topic}'
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_id}')
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Get actual camera properties
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera opened: {self.width}x{self.height} @ {self.actual_fps} FPS')
        
        # Create publishers
        self.image_pub = self.create_publisher(Image, output_topic, 10)
        self.visualization_pub = self.create_publisher(Image, visualization_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # Create timer for capturing frames
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize camera info message
        self.camera_info_msg = self.create_camera_info_msg()
        
        self.get_logger().info(f'Camera capture node started')
        self.get_logger().info(f'Publishing to: {output_topic}')
        self.get_logger().info(f'Visualization: {visualization_topic}')
        self.get_logger().info(f'Camera info: {camera_info_topic}')
    
    def create_camera_info_msg(self):
        """Create a basic camera info message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = self.width
        camera_info.height = self.height
        
        # Basic camera matrix (assuming no distortion)
        # fx, fy estimated from image size
        fx = fy = self.width
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        camera_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        camera_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        camera_info.distortion_model = "plumb_bob"
        
        return camera_info
    
    def timer_callback(self):
        """Capture and publish frame"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warning('Failed to capture frame')
            return
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Publish image
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id
            self.image_pub.publish(image_msg)
            
            # Also publish to visualization frame 1
            self.visualization_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')
        
        # Publish camera info
        self.camera_info_msg.header.stamp = timestamp
        self.camera_info_pub.publish(self.camera_info_msg)
    
    def destroy_node(self):
        """Clean up resources"""
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
