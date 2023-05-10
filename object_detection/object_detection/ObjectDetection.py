#! /usr/bin/env python3

import os
import importlib

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

from cv_bridge import CvBridge


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
                ('input_img_topic', 'color_camera/image_raw'),
                ('output_bb_topic', 'object_detection/img_bb'),
                ('output_img_topic', 'object_detection/img'),
                ('model_params.detector_type', 'YOLOv5'),
                ('model_params.model_dir_path', 'model/yolov5'),
                ('model_params.weight_file_name', 'auto_final.onnx'),
                ('model_params.confidence_threshold', 0.7),
                ('model_params.show_fps', 1),
                ('model_params.is_cuda', 0)
            ]
        )

        self.input_img_topic = self.get_parameter('input_img_topic').value
        self.output_bb_topic = self.get_parameter('output_bb_topic').value
        self.output_img_topic = self.get_parameter('output_img_topic').value
        
        
        self.detector_type = self.get_parameter('model_params.detector_type').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.weight_file_name = self.get_parameter('model_params.weight_file_name').value
        self.confidence_threshold = self.get_parameter('model_params.confidence_threshold').value
        self.show_fps = self.get_parameter('model_params.show_fps').value
        self.is_cuda = self.get_parameter('model_params.is_cuda').value
        
        # raise an exception if specified detector was not found
        if self.detector_type not in self.available_detectors:
            raise ModuleNotFoundError(self.detector_type + " Detector specified in config was not found. " + 
                                        "Check the Detectors dir for available detectors.")
        else:
            self.load_detector()
    
        
        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.bb_pub = None
        self.img_sub = self.create_subscription(Image, self.input_img_topic, self.detection_cb, 10)

        self.bridge = CvBridge()

    
    def discover_detectors(self):
        curr_dir = os.path.dirname(__file__)
        dir_contents = os.listdir(curr_dir + "/Detectors") 

        for entity in dir_contents:
            if entity.endswith('.py'):
                self.available_detectors.append(entity[:-3])

        self.available_detectors.remove('__init__')
        
    
    def load_detector(self):
        detector_mod = importlib.import_module(".Detectors." + self.detector_type, "object_detection")
        self.detector = detector_mod.register(self.model_dir_path, self.weight_file_name, 
                                              conf_threshold = self.confidence_threshold,
                                              is_cuda = self.is_cuda,
                                              show_fps = self.show_fps)
        
        print("Your detector : {} has been loaded !".format(self.detector_type))
    
    
    def detection_cb(self, img_msg):
        print("detection_cb")
        input = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        predictions, frame = self.detector.get_predictions(cv_image = input)

        if predictions == None :
            print("Image input from topic : {} is empty".format(self.input_img_topic))
        else :
            output = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.img_pub.publish(output)
        
            print(predictions)


def main():
    rclpy.init()
    od = ObjectDetection()
    try :
        rclpy.spin(od)

    except Exception as e:
        print(e)


if __name__=="__main__" :
    main()



