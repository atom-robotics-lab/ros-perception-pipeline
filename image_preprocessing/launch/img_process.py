# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

  pkg_perception_bringup = get_package_share_directory("perception_bringup")
  pkg_image_preprocessing = get_package_share_directory("image_preprocessing")

  #pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

  world_name = "playground"

  for arg in sys.argv:
      if arg.startswith("world:="):
        world_name = arg.split(":=")[1]

  world_sdf = pkg_perception_bringup + "/worlds/" + world_name + ".sdf"

  '''gz_sim = IncludeLaunchDescription( 
		PythonLaunchDescriptionSource(
		    os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
	)'''

  gz_sim_share = get_package_share_directory("ros_gz_sim")
  gz_sim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
      launch_arguments={
          "gz_args" : world_sdf
      }.items()
  )

  parameter_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", 
                          parameters = [
                             {'config_file'  : os.path.join(pkg_perception_bringup, "config", "bridge.yaml")}
                          ]
                      )
  
  image_process_node = Node(package="image_preprocessing",executable="image_process_node",
        output="screen",  # Adjust output options as needed
        parameters=[os.path.join(pkg_image_preprocessing, "config", "params.yaml")
        ]
    )

  arg_gz_sim = DeclareLaunchArgument('gz_args', default_value=world_sdf)

  arg_world_name = DeclareLaunchArgument('world', default_value='playground_world' )

  launch = [
    gz_sim,
    parameter_bridge,
    image_process_node,
  ]

  args = [
    arg_gz_sim,
    arg_world_name
  ]

  return LaunchDescription(args + launch)
