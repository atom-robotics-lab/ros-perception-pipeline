import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_image_preprocessing = get_package_share_directory("image_preprocessing")
  
  image_preprocess_node = Node(package="image_preprocessing",executable="image_process_node",
        output="screen",  # Adjust output options as needed
        parameters=[os.path.join(pkg_image_preprocessing, "config", "params.yaml")
        ]
    )

  launch = [
    image_preprocess_node,
  ]

  return LaunchDescription(launch)
