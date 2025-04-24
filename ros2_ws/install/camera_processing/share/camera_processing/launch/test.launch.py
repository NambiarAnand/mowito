from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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