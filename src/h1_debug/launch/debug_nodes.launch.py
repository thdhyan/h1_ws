from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for h1_debug nodes demonstration
    
    This launch file starts:
    1. Camera capture node - captures from camera
    2. Pose detection node - processes camera stream for pose detection
    3. Pose skeleton node - creates skeleton visualization with pelvis at origin
    4. Visualization grid node - displays multiple streams in a 2x2 grid
    """
    
    # Declare launch arguments
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    # Camera capture node
    camera_capture_node = Node(
        package='h1_debug',
        executable='camera_capture_node',
        name='camera_capture',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'output_topic': 'camera/image_raw',
            'visualization_topic': 'visualization/frame_1',
            'camera_info_topic': 'camera/camera_info',
            'namespace': '',
            'frame_id': 'camera_frame',
            'width': 640,
            'height': 480,
            'fps': 30
        }],
        output='screen'
    )
    
    # Pose detection node
    pose_detection_node = Node(
        package='h1_debug',
        executable='pose_detection_node',
        name='pose_detection',
        parameters=[{
            'input_topic': 'camera/image_raw',
            'output_pose_topic': 'pose/detection',
            'output_image_topic': 'pose/image_annotated',
            'visualization_topic': 'visualization/frame_4',  # Publish annotated image to frame_4
            'min_detection_confidence': 0.5,
            'min_tracking_confidence': 0.5,
            'model_complexity': 1,
            'enable_segmentation': False,
            'smooth_landmarks': True
        }],
        output='screen'
    )
    
    # Pose skeleton node
    pose_skeleton_node = Node(
        package='h1_debug',
        executable='pose_skeleton_node',
        name='pose_skeleton',
        parameters=[{
            'pose_topic': 'pose/detection',
            'visualization_topic': 'visualization/frame_2',
            'image_width': 640,
            'image_height': 480
        }],
        output='screen'
    )
    
    # Visualization grid node
    visualization_grid_node = Node(
        package='h1_debug',
        executable='visualization_grid_node',
        name='visualization_grid',
        parameters=[{
            'window_name': 'H1 Debug Visualization',
            'cell_width': 640,
            'cell_height': 480,
            'show_labels': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_id_arg,
        camera_capture_node,
        pose_detection_node,
        pose_skeleton_node,
        visualization_grid_node
    ])
