import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_launch = get_package_share_directory('image_launch')

    rviz_cfg_path = os.path.join(pkg_launch, 'rviz','rviz_cfg_base.rviz')

    return LaunchDescription([
        #Узел источник изображения
        Node(
            package='image_source',
            executable='image_source_node',
            name='virtual_camera_node',
            parameters=[{
                'source': 'video',
                'path': '/home/dmitry/Downloads/16198281792142 (1).mp4',
                'fps': 5
            }]
        ),
        # Узел обработчик изображения
        # Node(
        #     package='image_processing',
        #     executable='image_processing_node',
        #     name='image_processing_node'
        #     # parameters=[{
        #     #     'source': 'image',
        #     #     'path': '/home/ls/Pictures/Znaki-foto.jpg',
        #         # 'fps': 5
        #     # }]
        # ),
        # Node(
        #     package='image_processing',
        #     executable='image_processing_ml_knn_node',
        #     name='image_processing_ml_knn_node'
        # ),
        # Node(
        #     package='image_processing',
        #     executable='image_processing_ml_svm_node',
        #     name='image_processing_ml_svm_node'
        # ),
        #rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_visualizer',
            arguments=['-d',rviz_cfg_path]
        ),
    ])
