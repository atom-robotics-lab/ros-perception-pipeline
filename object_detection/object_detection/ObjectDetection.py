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

import cv2

from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        # Create an empty list that will hold the names of all available detector
        self.available_detectors = []

        # Create a logger instance
        self.logger  = super().get_logger()

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_img_topic', 'color/image_raw'),
                ('output_bb_topic', 'object_detection/img_bb'),
                ('output_img_topic', 'object_detection/img'),
                ('model_params.detector_type', 'YOLOv5'),
                ('model_params.model_dir_path', '/root/percep_ws/models/yolov5'),
                ('model_params.weight_file_name', 'yolov5.onnx'),
                ('model_params.confidence_threshold', 0.5),
                ('model_params.show_fps', 1),
            ]
        )
        # Load parameters set by user
        self.load_parameters()

        # Fill available_detectors with the detectors from Detectors directory
        self.discover_detectors()
        # Load the detector specified through the parameters
        self.load_detector()

        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.bb_pub = None
        self.img_sub = self.create_subscription(Image, self.input_img_topic, self.detection_cb, 10)

        self.bridge = CvBridge()

        self.logger.info("[OBJECT DETECTION] Initialized Object Detection Node")

    def load_parameters(self):
        # Node params
        self.input_img_topic = self.get_parameter('input_img_topic').value
        self.output_bb_topic = self.get_parameter('output_bb_topic').value
        self.output_img_topic = self.get_parameter('output_img_topic').value

        self.logger.info("[OBJECT DETECTION] Input image topic set to {}".format(self.input_img_topic))
        self.logger.info("[OBJECT DETECTION] Publishig output image on topic {}".format(self.output_img_topic) +
                         " and bounding box data on topic {}".format(self.output_bb_topic))

        # Model params
        self.detector_type = self.get_parameter('model_params.detector_type').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.weight_file_name = self.get_parameter('model_params.weight_file_name').value
        self.confidence_threshold = self.get_parameter('model_params.confidence_threshold').value
        self.show_fps = self.get_parameter('model_params.show_fps').value

        self.logger.info("[OBJECT DETECTION] Detector type set to {} and".format(self.detector_type) +
                         " using weigth file from {}".format(self.model_dir_path + self.weight_file_name))
        self.logger.info("[OBJECT DETECTION] Confidence threshold for detection set to: {}".format(self.confidence_threshold))

    def discover_detectors(self):
        curr_dir = os.path.dirname(__file__)
        dir_contents = os.listdir(curr_dir + "/Detectors")

        for entity in dir_contents:
            if entity.endswith('.py'):
                self.available_detectors.append(entity[:-3])

        self.available_detectors.remove('__init__')

    def load_detector(self):
        # Raise an exception if specified detector was not found
        if self.detector_type not in self.available_detectors:
            self.logger.error("[OBJECT DETECTION] {} Detector was set in parameters but was not found. Check the " +
                              "Detectors directory for list of available detectors".format(self.detector_type))
            raise ModuleNotFoundError(self.detector_type + " Detector specified in config was not found. " +
                                      "Check the Detectors dir for available detectors.")
        else:
            detector_mod = importlib.import_module(".Detectors." + self.detector_type,
                                                   "object_detection")
            detector_class = getattr(detector_mod, self.detector_type)
            self.detector = detector_class(self.logger)

            self.detector.build_model(self.model_dir_path, self.weight_file_name)
            self.detector.load_classes(self.model_dir_path)

            self.logger.info("[OBJECT DETECTION] Your detector: {} has been loaded!".format(self.detector_type))

    def detection_cb(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        predictions = self.detector.get_predictions(cv_image=cv_image)

        if predictions is None:
            self.logger.warning("[OBJECT DETECTION] Image input from topic: {} is empty".format(self.input_img_topic), throttle_duration_sec=1)
        else:
            for prediction in predictions:
                confidence = prediction['confidence']

                # Check if the confidence is above the threshold
                if confidence >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, prediction['box'])

                    # Draw the bounding box
                    cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 1)

                    # Show names of classes on the output image
                    class_id = int(prediction['class_id'])
                    class_name = self.detector.class_list[class_id]
                    label = f"{class_name} : {confidence:.2f}"

                    cv_image = cv2.putText(cv_image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Publish the modified image
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
