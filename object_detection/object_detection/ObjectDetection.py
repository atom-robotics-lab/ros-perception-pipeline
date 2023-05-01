#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D


from Detectors import YOLOv5, YOLOv8, EfficientDet, RetinaNet 

from cv_bridge import CvBridge


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('output_img_topic', 'object_detection/img_bb'),
                ('model_params.detector_type', 'YOLOv5'),
                ('model_params.model_dir_path', 'model'),
                ('model_name', "auto_final.onnx")
            ]
        )

        self.output_img_topic = self.get_parameter('output_img_topic').value
        self.detector_type = self.get_parameter('model_name').value
        self.model_dir_path = self.get_parameter('model_params.model_dir_path').value
        self.model_name = self.get_parameter('model_name').value


        if self.detector_type == "YOLOv5" :
            self.detector = YOLOv5.YOLOv5()
        elif self.detector_type == "YOLOv8" :
            self.detector = YOLOv8.YOLOv8()
        elif self.detector_type == "RetinaNet" :
            self.detector = RetinaNet.RetinaNet()
        elif self.detector_type == "EfficientDet" :
            self.detector = EfficientDet.EfficientDet()
        else :
            rclpy.logerr("The detector type : {} is not supported".format(self.detector_type))

        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.img_sub = self.create_subscription(Image, 'img_raw', self.detection_cb, 10)

        self.bridge = CvBridge()

    def detection_cb(self, img_msg):
        input = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        predictions = self.detector.get_predictions(cv_image = input)

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



