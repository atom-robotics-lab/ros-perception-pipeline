# Copyright (c) 2023 A.T.O.M ROBOTICS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_object_detection = get_package_share_directory("object_detection")

    params = os.path.join(
        pkg_object_detection,
        'config',
        'params.yaml'
        )

    node = Node(
        package='object_detection',
        name='object_detection',
        executable='ObjectDetection',
        parameters=[params],
        output="screen",
        emulate_tty=True
    )

    return LaunchDescription([node])
