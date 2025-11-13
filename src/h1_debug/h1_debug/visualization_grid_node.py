#!/usr/bin/env python3
"""
Visualization Grid Node
Creates a 2x2 grid visualization from 4 input image topics
Subscribes to /visualization/frame_{1-4} topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisualizationGridNode(Node):
    def __init__(self):
        super().__init__('visualization_grid_node')
        
        # Declare parameters
        self.declare_parameter('window_name', 'Visualization Grid')
        self.declare_parameter('cell_width', 640)
        self.declare_parameter('cell_height', 480)
        self.declare_parameter('show_labels', True)
        
        # Get parameters
        self.window_name = self.get_parameter('window_name').value
        self.cell_width = self.get_parameter('cell_width').value
        self.cell_height = self.get_parameter('cell_height').value
        self.show_labels = self.get_parameter('show_labels').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize frame storage
        self.frames = {
            1: None,
            2: None,
            3: None,
            4: None
        }
        
        # Create subscribers for each frame
        self.subscribers = []
        for i in range(1, 5):
            topic = f'/visualization/frame_{i}'
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, frame_id=i: self.image_callback(msg, frame_id),
                10
            )
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')
        
        # Create timer for visualization update
        self.timer = self.create_timer(0.033, self.update_visualization)  # ~30 FPS
        
        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        self.get_logger().info('Visualization grid node started')
    
    def image_callback(self, msg, frame_id):
        """Callback for receiving images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize to cell size
            resized = cv2.resize(cv_image, (self.cell_width, self.cell_height))
            
            # Store frame
            self.frames[frame_id] = resized
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame {frame_id}: {str(e)}')
    
    def update_visualization(self):
        """Update the visualization grid"""
        # Create placeholder for missing frames
        placeholder = np.zeros((self.cell_height, self.cell_width, 3), dtype=np.uint8)
        
        # Prepare frames for grid
        grid_frames = []
        for i in range(1, 5):
            frame = self.frames[i]
            if frame is None:
                frame = placeholder.copy()
                # Add "No Signal" text
                cv2.putText(frame, f'Frame {i}', (self.cell_width//2 - 50, self.cell_height//2),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)
                cv2.putText(frame, 'No Signal', (self.cell_width//2 - 70, self.cell_height//2 + 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            else:
                frame = frame.copy()
            
            # Add label if enabled
            if self.show_labels:
                cv2.putText(frame, f'Frame {i}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            grid_frames.append(frame)
        
        # Create 2x2 grid
        top_row = np.hstack([grid_frames[0], grid_frames[1]])
        bottom_row = np.hstack([grid_frames[2], grid_frames[3]])
        grid = np.vstack([top_row, bottom_row])
        
        # Add border between cells
        # Vertical line
        cv2.line(grid, (self.cell_width, 0), (self.cell_width, self.cell_height * 2),
                (255, 255, 255), 2)
        # Horizontal line
        cv2.line(grid, (0, self.cell_height), (self.cell_width * 2, self.cell_height),
                (255, 255, 255), 2)
        
        # Display grid
        cv2.imshow(self.window_name, grid)
        cv2.waitKey(1)
    
    def destroy_node(self):
        """Clean up resources"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationGridNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
