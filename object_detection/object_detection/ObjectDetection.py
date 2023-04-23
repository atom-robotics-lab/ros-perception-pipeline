#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D


from .Detectors.YOLOv5 import YOLOv5

from cv_bridge import CvBridge


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('output_img_topic', 'object_detection/img_bb'),
                ('model_params.detector', 'YOLOv5'),
                ('model_params.model_dir_path', None)
            ]
        )

        self.output_img_topic = self.get_parameter('output_img_topic').value

        self.img_pub = self.create_publisher(Image, self.output_img_topic, 10)
        self.img_sub = self.create_subscription(Image, 'img_raw', self.detection_cb, 10)

        self.bridge = CvBridge()

    def detection_cb(self, img_msg):
        input = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        self.detector = YOLOv5(model_dir_path = 'model', 
                               conf_threshold = 0.7, 
                               score_threshold = 0.4, 
                               nms_threshold = 0.25, 
                               is_cuda = 0)
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



