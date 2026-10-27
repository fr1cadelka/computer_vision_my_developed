from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Нода загрузки облака из файла (.ply)
    cloud_loader = Node(
        package="cloud_loader",
        executable="cloud_loader_node",
        name="cloud_loader_node"
    )

    # 2. Нода очистки облака от шума (Voxel + Statistical)
    cloud_filter = Node(
        package="cloud_filter",
        executable="cloud_filter_node",
        name="cloud_filter_node"
    )

    # 3. Нода поиска плоскости (RANSAC)
    plane_segmentation = Node(
        package="plane_segmentation",
        executable="plane_segmentation_node",
        name="plane_segmentation_node",
        parameters=[{
            # Тут твои параметры для обрезки "коробки"
            "x_min":  0.0,
            "x_max":  25.0,
            "y_min":  3.0,
            "y_max":  8.0,
            "z_min":  0.2,
            "z_max":  0.9,
            "ransac_threshold": 0.4
            # се точки, которые находятся в пределах 40 сантиметров от найденной плоскости
        }]
    )

    # 4. Запуск RViz с заранее сохраненным конфигом
    pkg_share = get_package_share_directory("cloud_launch")
    rviz_config_path = os.path.join(pkg_share, "rviz", "plane_config.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_path] # Флаг -d загружает конфиг
    )

    # Возвращаем список всех запускаемых нод
    return LaunchDescription([
        cloud_loader,
        cloud_filter,
        plane_segmentation,
        rviz
    ])
