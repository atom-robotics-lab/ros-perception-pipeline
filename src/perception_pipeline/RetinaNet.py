#!/usr/bin/env python3


from tensorflow import keras
from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


class RetinaNet:
    def __init__(self):
        # self.image = 0
        self.path = os.path.dirname(__file__)[:-7]
        self.model_path = os.path.join('..', 'snapshots', f"{self.path}scripts/utils/resnet50_coco_best_v2.1.0.h5")
        self.model = models.load_model(self.model_path, backbone_name='resnet50')
        self.labels_to_names = self.load_classes()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.load_capture)    
        


    def load_classes(self):
        self.class_list = []
        with open(f"{self.path}scripts/utils/coco_classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list

    def load_capture(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data)
        try:
            self.image = (self.frame)
            self.object_detection()                    
        except:
            pass
        
    def object_detection(self):        

        # copy to draw on
        draw = self.image.copy()
        # preprocess image for network
        image = preprocess_image(self.image)
        image, scale = resize_image(image)

        # process image
        start = time.time()
        boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(image, axis=0))
        print("processing time: ", time.time() - start)

        # correct for image scale
        boxes /= scale

        # visualize detections
        for box, score, label in zip(boxes[0], scores[0], labels[0]):
            # scores are sorted so we can break
            if score < 0.5:
                break
                
            color = label_color(label)
            
            b = box.astype(int)
            draw_box(draw, b, color=color)
            
            caption = "{} {:.3f}".format(self.labels_to_names[label], score)
            print(self.labels_to_names[label])
            draw_caption(draw, b, caption)
        
        while True:
            cv2.imshow("ok", draw)
            cv2.waitKey(0)       
        
        

if __name__ == '__main__' :
    rospy.init_node("preception_retinanet")

    ok = RetinaNet()
    rospy.spin()    
