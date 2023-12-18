#! /usr/bin/env python3

import os
import importlib

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
#from vision_msgs.msg import BoundingBox2D

from cv_bridge import CvBridge
import cv2


class InstanceSegmentation(Node):
    def __init__(self):
        super().__init__('instance_segmentation')

        # create an empty list that will hold the names of all available segmentators
        self.available_segmentators = []
        
        # fill available_segmentators with the segmentators from Segmentators dir
        self.discover_segmentators()

        self.declare_parameters(
            namespace='',
            parameters=[

                ('input_img_topic', ""),
                ('output_bb_topic', ""),
                ('output_img_topic', ""),
                ('model_params.segmentator_type', ""),
                ('model_params.model_dir_path', ""),
                ('model_params.weight_file_name', ""),
                ('model_params.confidence_threshold', 0.7),
                ('model_params.show_fps', 1),
            ]
        )

        # node params
        self.input_img_topic = self.get_parameter('input_img_topic').value
        self.output_bb_topic = self.get_parameter('output_bb_topic').value
        self.output_img_topic = self.get_parameter('output_img_topic').value
        
        # model params
        self.segmentator_type = self.get_parameter('model_params.segmentator_type').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.weight_file_name = self.get_parameter('model_params.weight_file_name').value
        self.confidence_threshold = self.get_parameter('model_params.confidence_threshold').value
        self.show_fps = self.get_parameter('model_params.show_fps').value
        
        print(self.segmentator_type)
        # raise an exception if specified segmentator was not found
        if self.segmentator_type not in self.available_segmentators:
            raise ModuleNotFoundError(self.segmentator_type + " Segmentator specified in config was not found. " + 
                                        "Check the Segmentators dir for available segmentators.")
        else:
            self.load_segmentator()

        self.load_segmentator()
        
        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.bb_pub = None
        # self.img_sub = self.create_subscription(Image, self.input_img_topic, self.segmentation_cb, 10)
        self.img_sub = self.create_subscription(Image, self.input_img_topic, self.segmentation_image, 10)

        self.bridge = CvBridge()

    
    def discover_segmentators(self):
        curr_dir = os.path.dirname(__file__)
        dir_contents = os.listdir(curr_dir + "/Segmentators") 

        for entity in dir_contents:
            if entity.endswith('.py'):
                self.available_segmentators.append(entity[:-3])

        print(self.available_segmentators)

        self.available_segmentators.remove('__init__')
        
    
    def load_segmentator(self):
        segmentator_mod = importlib.import_module(".Segmentators." + self.segmentator_type, "instance_segmentation")
        segmentator_class = getattr(segmentator_mod, self.segmentator_type)
        self.segmentator = segmentator_class(self.model_dir_path, self.weight_file_name)
        
        self.segmentator.build_model()
        self.segmentator.load_classes()

        print("Your segmentator : {} has been loaded !".format(self.segmentator_type))
    
    
    def segmentation_cb(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        predictions = self.segmentator.get_predictions(cv_image=cv_image)

        if predictions == None :
            print("Image input from topic : {} is empty".format(self.input_img_topic))
        else :
            for prediction in predictions:
                left, top, width, height = prediction['box']
                right = left + width
                bottom = top + height

                # Draw the bounding box
                cv_image = cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 0), 1)

            output = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.img_pub.publish(output)
            print(predictions)

    def segmentation_image(self, img_msg):
        # print(img_msg.value)
        # cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        # segmented_image = self.segmentator.get_segmented_image(cv_image)
        segmented_image = self.segmentator.get_segmented_image(img_msg)
        output = self.bridge.cv2_to_imgmsg(segmented_image, "bgr8")
        self.img_pub.publish(output)
        cv2.imshow(segmented_image)


def main():
    rclpy.init()
    iseg = InstanceSegmentation()
    try:
        rclpy.spin(iseg)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
