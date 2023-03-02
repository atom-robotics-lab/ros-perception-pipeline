#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


if __name__ == "__main__":
    rospy.init_node("Yolo_Test_Node")
    img_pub = rospy.Publisher("/camera/color/image_raw2", Image)
    img = cv2.imread("../images/test_image.png")
    print(type(img))
    img_bridge = CvBridge()

    ros_img = img_bridge.cv2_to_imgmsg(img, encoding='bgr8')

    while True:
        img_pub.publish(ros_img)