import rclpy
from rclpy.Node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from Detectors import *


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

        self.img_pub = self.create_publisher(Image, self.output_img_topic)
        self.img_sub = self.create_subscription(Image, 'img_raw', self.detection_cb)

    def detection_cb(self, img_msg):

