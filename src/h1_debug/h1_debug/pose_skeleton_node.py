#!/usr/bin/env python3
"""
Pose Skeleton Visualization Node
Subscribes to pose detection messages and creates a 3D skeleton visualization
with pelvis fixed at origin, published to visualization frame 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from h1_msgs.msg import PoseDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import io


class PoseSkeletonNode(Node):
    def __init__(self):
        super().__init__('pose_skeleton_node')

        # Declare parameters
        self.declare_parameter('pose_topic', 'pose/detection')
        self.declare_parameter('visualization_topic', 'visualization/frame_2')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Get parameters
        pose_topic = self.get_parameter('pose_topic').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # MediaPipe pose connections (simplified for main body)
        # These are the key connections for a basic skeleton
        self.pose_connections = [
            # Torso
            (11, 12),  # shoulders
            (11, 23),  # left shoulder to left hip
            (12, 24),  # right shoulder to right hip
            (23, 24),  # hips

            # Left arm
            (11, 13),  # left shoulder to left elbow
            (13, 15),  # left elbow to left wrist

            # Right arm
            (12, 14),  # right shoulder to right elbow
            (14, 16),  # right elbow to right wrist

            # Left leg
            (23, 25),  # left hip to left knee
            (25, 27),  # left knee to left ankle

            # Right leg
            (24, 26),  # right hip to right knee
            (26, 28),  # right knee to right ankle
        ]

        # Create subscriber
        self.pose_sub = self.create_subscription(
            PoseDetection,
            pose_topic,
            self.pose_callback,
            10
        )

        # Create publisher
        self.image_pub = self.create_publisher(Image, self.visualization_topic, 10)

        self.get_logger().info(f'Pose skeleton node started')
        self.get_logger().info(f'Subscribing to: {pose_topic}')
        self.get_logger().info(f'Publishing 3D view to: {self.visualization_topic}')

    def create_3d_plot(self, normalized_landmarks):
        """Create a 3D matplotlib plot of the skeleton and return as OpenCV image"""
        # Create figure with specific size to match our image dimensions
        fig = plt.figure(figsize=(self.image_width/100, self.image_height/100), dpi=100)
        ax = fig.add_subplot(111, projection='3d')

        # Set up the plot
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Pose Skeleton')

        # Set orthogonal projection
        ax.set_proj_type('ortho')

        # Set background color to match OpenCV (black)
        ax.set_facecolor('black')
        fig.patch.set_facecolor('black')

        # Set axis colors to white for visibility
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.zaxis.label.set_color('white')
        ax.tick_params(colors='white')
        ax.title.set_color('white')

        # Remove grid and panes for cleaner look
        ax.grid(False)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')

        if normalized_landmarks:
            # Extract coordinates
            x_coords = [landmark[0] for landmark in normalized_landmarks]
            y_coords = [landmark[1] for landmark in normalized_landmarks]
            z_coords = [landmark[2] for landmark in normalized_landmarks]

            # Plot landmarks as scatter points
            ax.scatter(x_coords, y_coords, z_coords, c='lime', s=50, alpha=0.8)

            # Draw skeleton connections
            for connection in self.pose_connections:
                if connection[0] < len(normalized_landmarks) and connection[1] < len(normalized_landmarks):
                    pt1 = normalized_landmarks[connection[0]]
                    pt2 = normalized_landmarks[connection[1]]
                    ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], [pt1[2], pt2[2]],
                           color='lime', linewidth=3, alpha=0.8)

        # Convert matplotlib figure to OpenCV image
        buf = io.BytesIO()
        fig.savefig(buf, format='png', bbox_inches='tight', facecolor='black')
        buf.seek(0)
        plt.close(fig)

        # Convert PNG buffer to OpenCV image
        png_data = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        cv_image = cv2.imdecode(png_data, cv2.IMREAD_COLOR)

        # Resize to match our target dimensions if needed
        if cv_image.shape[0] != self.image_height or cv_image.shape[1] != self.image_width:
            cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))

        return cv_image

    def pose_callback(self, msg):
        """Process pose detection message and create 3D skeleton visualization"""
        try:
            # Extract and normalize landmarks
            normalized_landmarks = []
            if msg.detected and len(msg.landmarks) >= 33:
                # Extract landmarks
                landmarks = []
                for landmark in msg.landmarks:
                    landmarks.append((landmark.x, landmark.y, landmark.z))

                # Normalize pose so pelvis (landmark 23/24 midpoint) is at origin
                if len(landmarks) > 24:
                    pelvis_x = (landmarks[23][0] + landmarks[24][0]) / 2.0
                    pelvis_y = (landmarks[23][1] + landmarks[24][1]) / 2.0
                    pelvis_z = (landmarks[23][2] + landmarks[24][2]) / 2.0

                    # Center all landmarks around pelvis
                    for landmark in landmarks:
                        norm_x = landmark[0] - pelvis_x
                        norm_y = landmark[1] - pelvis_y
                        norm_z = landmark[2] - pelvis_z
                        normalized_landmarks.append((norm_x, norm_y, norm_z))
                else:
                    normalized_landmarks = landmarks

            # Create 3D visualization
            skeleton_image = self.create_3d_plot(normalized_landmarks)

            # Publish the 3D visualization
            try:
                ros_image = self.bridge.cv2_to_imgmsg(skeleton_image, encoding='bgr8')
                ros_image.header = msg.header
                ros_image.header.frame_id = 'pose_skeleton_3d'
                self.image_pub.publish(ros_image)

            except Exception as e:
                self.get_logger().error(f'Error publishing skeleton image: {str(e)}')

        except Exception as e:
            self.get_logger().error(f'Error in pose skeleton callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSkeletonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
