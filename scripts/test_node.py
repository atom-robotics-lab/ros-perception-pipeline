#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import os



if __name__ == "__main__":
    rospy.init_node("Yolo_Test_Node")
    path = os.path.dirname(__file__)[:-7]
    img_pub = rospy.Publisher("/camera/color/image_raw2", Image)
    img = cv2.imread(f"{path}/images/RetinaNet_image_test.jpeg")
    img_bridge = CvBridge()


    ros_img = img_bridge.cv2_to_imgmsg(img, encoding='bgr8')

    while True:
        img_pub.publish(ros_img)