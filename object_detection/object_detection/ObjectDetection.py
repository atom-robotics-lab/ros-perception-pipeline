#! /usr/bin/env python3

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

import importlib
import os
import time

import cv2

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        # create an empty list that will hold the names of all available detector
        self.available_detectors = []

        # fill available_detectors with the detectors from Detectors dir
        self.discover_detectors()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_img_topic', ''),
                ('output_bb_topic', ''),
                ('output_img_topic', ''),
                ('model_params.detector_type', ''),
                ('model_params.model_dir_path', ''),
                ('model_params.weight_file_name', ''),
                ('model_params.confidence_threshold', 0.7),
                ('model_params.show_fps', 1),
            ]
        )

        # node params
        self.input_img_topic = self.get_parameter('input_img_topic').value
        self.output_bb_topic = self.get_parameter('output_bb_topic').value
        self.output_img_topic = self.get_parameter('output_img_topic').value

        # model params
        self.detector_type = self.get_parameter('model_params.detector_type').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.weight_file_name = self.get_parameter('model_params.weight_file_name').value
        self.confidence_threshold = self.get_parameter('model_params.confidence_threshold').value
        self.show_fps = self.get_parameter('model_params.show_fps').value

        # Load the detector
        self.load_detector()

        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.bb_pub = None
        self.img_sub = self.create_subscription(Image, self.input_img_topic, self.detection_cb, 10)

        self.bridge = CvBridge()

        if self.show_fps:
            self.start_time = time.time()
            self.frame_count = 0
            self.total_elapsed_time = 0
            self.average_fps = 0

    def discover_detectors(self):
        curr_dir = os.path.dirname(__file__)
        dir_contents = os.listdir(curr_dir + "/Detectors")

        for entity in dir_contents:
            if entity.endswith('.py'):
                self.available_detectors.append(entity[:-3])

        self.available_detectors.remove('__init__')

    def load_detector(self):
        for detector_name in self.available_detectors:
            if self.detector_type.lower() == detector_name.lower():

                detector_mod = importlib.import_module(".Detectors." + detector_name,
                                                       "object_detection")
                detector_class = getattr(detector_mod, detector_name)
                self.detector = detector_class()

                self.detector.build_model(self.model_dir_path, self.weight_file_name)
                self.detector.load_classes(self.model_dir_path)

                print("Your detector: {} has been loaded !".format(detector_name))
                return

        raise ModuleNotFoundError(self.detector_type + " Detector specified in config was not found. " +
                                  "Check the Detectors dir for available detectors.")

    def detection_cb(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        start_time = time.time()

        predictions = self.detector.get_predictions(cv_image=cv_image)

        if predictions is None:
            print("Image input from topic: {} is empty".format(self.input_img_topic))
        else:
            for prediction in predictions:
                x1, y1, x2, y2 = map(int, prediction['box'])

                # Draw the bounding box
                cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 1)

                # Show names of classes on the output image
                class_id = int(prediction['class_id'])
                class_name = self.detector.class_list[class_id]
                label = f"{class_name}: {prediction['confidence']:.2f}"

                cv_image = cv2.putText(cv_image, label, (x1, y1 - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if self.show_fps:
                elapsed_time = time.time() - start_time
                self.frame_count += 1
                self.total_elapsed_time += elapsed_time

                # Write FPS on the frame
                cv_image = cv2.putText(cv_image, f"FPS: {self.average_fps:.2f}", (10, 30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if time.time() - self.start_time >= 1.0:
                    self.average_fps = self.frame_count / self.total_elapsed_time

                    self.frame_count = 0
                    self.total_elapsed_time = 0
                    self.start_time = time.time()

            output = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.img_pub.publish(output)


def main():
    rclpy.init()
    od = ObjectDetection()
    try:
        rclpy.spin(od)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
