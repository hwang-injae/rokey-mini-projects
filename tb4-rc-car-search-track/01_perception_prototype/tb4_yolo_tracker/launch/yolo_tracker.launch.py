from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tb4_yolo_tracker',
            executable='yolo_tracker',        # setup.py console_scripts 이름
            name='yolo_tracker',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'image_topic': '/oakd/rgb/preview/image_raw',
                'image_transport': 'raw',
                'conf': 0.25,
                'tracker': 'bytetrack.yaml',
                'device': 'cpu',
                'imgsz': 320,
                'classes': '',
                'publish_annotated': True,
                'inference_rate': 8.0,
            }],
        ),
    ])
