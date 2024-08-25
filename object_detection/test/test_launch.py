# #! /usr/bin/env python3

# import time
# import unittest
# import os
# import pytest

# import launch
# import launch_ros
# import launch.actions
# import launch_testing.actions
# import launch_testing.markers
# from launch.actions import ExecuteProcess
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image

# from ament_index_python.packages import get_package_share_directory


# def wait_for_node(test_node, node_name, timeout):

#     start = time.time()
#     flag = False
#     print("Waiting for {}".format(node_name))

#     while time.time() - start < timeout and not flag:
#         flag = node_name in test_node.get_node_names()
    
#     return flag

# def is_topic_active(test_node, target_topic_name, timeout):

#     start = time.time()
#     flag = False
#     print("Waiting for {}".format(target_topic_name))

#     unique_topics = []

#     while time.time() - start < timeout and not flag:
#         topic_list = test_node.get_topic_names_and_types()
#         for topic_name, types in topic_list:

#             if topic_name not in unique_topics:
#                 unique_topics.append(topic_name)

#             #print("\nTOPIC : {}\n".format(topic_name))
#             if target_topic_name == topic_name:
#                 flag = True
#     print("\n unique topics : ", unique_topics)

#     return flag


# @pytest.mark.launch_test
# @launch_testing.markers.keep_alive
# def generate_test_description():

#     object_detection_pkg_dir = get_package_share_directory("object_detection")

#     # Path to the parameters file
#     test_params_file_path = os.path.join(object_detection_pkg_dir, "test", "test_config", "test_params.yaml")
    
#     # Define the ObjectDetection node with the parameters file
#     object_detection_node = launch_ros.actions.Node(
#         package="object_detection",
#         executable="ObjectDetection",
#         output="screen",
#         parameters=[test_params_file_path]
#     )

#     test_bag_path = os.path.join(object_detection_pkg_dir, "test")
#     test_bag_proc = ExecuteProcess(
#         cmd=["ros2", "bag", "play", f"{test_bag_path}/test_bag", "-l"],
#     )

#     return launch.LaunchDescription([
#         object_detection_node,
#         test_bag_proc,
#         launch_testing.actions.ReadyToTest()
#     ])


# class TestPresence(unittest.TestCase):
#     def setUp(self):
#         rclpy.init()
#         self.test_node = Node("test_node")

#     def test_node_presence(self, proc_output):
#         assert wait_for_node(self.test_node, "object_detection", 8.0), "object_detection Node not found !"
 
#     def test_topic_presence(self, proc_output):
#         assert is_topic_active(self.test_node, "/test_topic/img", 8.0), "/object_detection/img topic not found !"

#     def tearDown(self):
#         self.test_node.destroy_node()
#         rclpy.shutdown()


# class TestImageMessage(unittest.TestCase):
#     def setUp(self):
#         rclpy.init()
#         self.test_node = Node("test_node")

#         self.img_cb_flag = False
#         self.img_msg = None

#         # Create the subscription
#         self.subscription = self.test_node.create_subscription(
#             Image,
#             '/test_topic/img',
#             self.img_msg_cb,
#             1
#         )

#     def test_img_msg(self):
#         # Allow some time for messages to be received
#         start = time.time()

#         while time.time() - start < 10.0:
#             rclpy.spin_once(self.test_node, timeout_sec=0.1)
#             if self.img_cb_flag:
#                 break

#         # Assert that the callback was called
#         self.assertTrue(self.img_cb_flag, "Callback function was not called within the time limit.")

#     def img_msg_cb(self, img_msg):
#         # This method will be called when a message is received
#         self.img_msg = img_msg
#         self.img_cb_flag = True

#     def tearDown(self):
#         self.test_node.destroy_node()
#         rclpy.shutdown()


# # @launch_testing.post_shutdown_test()
# # class TestNodeShutdown(unittest.TestCase):
# #     def test_node_exit_code(self, proc_info, proc_output):
# #         # Verify that the node shut down properly
# #         launch_testing.asserts.assertExitCodes(proc_info, process='ObjectDetection')
