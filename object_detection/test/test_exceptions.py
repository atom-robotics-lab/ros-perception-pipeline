#! /usr/bin/env python3

import time
import unittest
import os
import pytest

import launch
import launch_ros
import launch.actions
import launch_testing.actions
import launch_testing.markers
from launch.actions import ExecuteProcess
from launch_testing.asserts import assertExitCodes, assertInStdout
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory

def wait_for_node(test_node, node_name, timeout):

    start = time.time()
    flag = False
    print("Waiting for {}".format(node_name))

    while time.time() - start < timeout and not flag:
        flag = node_name in test_node.get_node_names()
    
    return flag


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():

    object_detection_pkg_dir = get_package_share_directory("object_detection")

    # Path to the parameters file
    test_params_file_path = os.path.join(object_detection_pkg_dir, "test", "test_config", "test_exceptions.yaml")
    
    # Define the ObjectDetection node with the parameters file
    object_detection_node = launch_ros.actions.Node(
        package="object_detection",
        executable="ObjectDetection",
        output="screen",
        parameters=[test_params_file_path]
    )

    test_bag_path = os.path.join(object_detection_pkg_dir, "test")
    test_bag_proc = ExecuteProcess(
        cmd=["ros2", "bag", "play", f"{test_bag_path}/test_bag", "-l"],
    )

    return launch.LaunchDescription([
        object_detection_node,
        test_bag_proc,
        launch_testing.actions.ReadyToTest()
    ]), {'object_detection_node': object_detection_node}

class DelayShutdown(unittest.TestCase):
    # Wait for 10 seconds for the GTest script to complete
    def test_delay(self):
        time.sleep(10)

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info, object_detection_node):
        assertExitCodes(proc_info, process=object_detection_node, allowable_exit_codes=[1])

    def test_module_not_found_error(self, proc_output, object_detection_node):
        expected_error = "ModuleNotFoundError"
        test_detector = "TestDetector"

        # Capture stderr output
        stderr_output = ""
        for event in proc_output[object_detection_node]:
            if isinstance(event, launch.events.process.ProcessStderr):
                stderr_output += event.text.decode('utf-8')

        # # Log the captured stderr
        # with open('logs.txt', 'w') as f:
        #     f.write(f"Captured stderr:\n{stderr_output}\n")

        # Check if the expected error is in the stderr output
        self.assertIn(expected_error, stderr_output, f"Expected '{expected_error}' not found in stderr")

        # You can add more specific checks here, e.g.:
        self.assertIn(f"{test_detector} Detector specified in config was not found", stderr_output, "TestDetector not found in stderr")
