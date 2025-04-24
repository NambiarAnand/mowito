from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to default image
    image_path = os.path.join(
        get_package_share_directory('camera_processing'),
        'resources',
        'test_image.jpg'
    )
    
    return LaunchDescription([
        # Dummy USB Camera Node
        Node(
            package='camera_processing',
            executable='dummy_usb_cam',
            name='dummy_usb_cam',
            output='screen',
            parameters=[{
                'image_filename': 'test_image.jpg',
                'frame_rate': 30,
                'topic_name': '/image_raw'
            }]
        ),
        
        # Image Conversion Node
        Node(
            package='camera_processing',
            executable='image_conversion',
            name='image_conversion',
            output='screen',
            parameters=[{
                'input_topic': '/image_raw',
                'output_topic': '/converted_image'
            }]
        )
    ])