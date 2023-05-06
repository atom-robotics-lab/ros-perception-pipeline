#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D


from .Detectors import YOLOv5, YOLOv8, EfficientDet, RetinaNet 

from cv_bridge import CvBridge


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_img_topic', 'color_camera/img_raw'),
                ('output_bb_topic', 'object_detection/img_bb'),
                ('output_img_topic', 'object_detection/img'),
                ('model_params.detector_type', 'YOLOv5'),
                ('model_params.model_dir_path', 'model/yolov5'),
                ('model_params.weight_file_name', "auto_final.onnx")
            ]
        )

        self.input_img_topic = self.get_parameter('input_img_topic').value
        self.output_bb_topic = self.get_parameter('output_bb_topic').value
        self.output_img_topic = self.get_parameter('output_img_topic').value
        self.detector_type = self.get_parameter('model_params.detector_type').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.weight_file_name = self.get_parameter('model_params.weight_file_name').value


        if self.detector_type == "YOLOv5" :
            print("Using detector : {}".format(self.detector_type))
            self.detector = YOLOv5.YOLOv5(self.model_dir_path, self.weight_file_name)

        elif self.detector_type == "YOLOv8" :
            print("Using detector : {}".format(self.detector_type))
            self.detector = YOLOv8.YOLOv8(self.model_dir_path, self.weight_file_name)

        elif self.detector_type == "RetinaNet" :
            print("Using detector : {}".format(self.detector_type))
            self.detector = RetinaNet.RetinaNet(self.model_dir_path, self.weight_file_name)

        elif self.detector_type == "EfficientDet" :
            print("Using detector : {}".format(self.detector_type))
            self.detector = EfficientDet.EfficientDet(self.model_dir_path, self.weight_file_name)

        else :
            print("The detector type : {} is not supported".format(self.detector_type))

        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.bb_pub = None

        self.img_sub = self.create_subscription(Image, self.input_img_topic, self.detection_cb, 10)

        self.bridge = CvBridge()

    def detection_cb(self, img_msg):
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



