#! /usr/bin/env python

from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time
import matplotlib.pyplot as plt

from ..DetectorBase import DetectorBase

class RetinaNet_plugin(DetectorBase) :
    def __init(self, model_dir_path, weight_file_name, conf_threshold = 0.7, 
                 score_threshold = 0.4, nms_threshold = 0.25, is_cuda = 0, show_fps = 1) :
        super.__init__()

        self.predictions = []

        self.model_dir_path = model_dir_path
        self.weight_file_name = weight_file_name

        self.predictions = []
        self.conf_threshold = conf_threshold
        self.show_fps = show_fps
        self.is_cuda = is_cuda

    def build_model(self) :
        try :
            self.model_path = os.path.join(self.model_dir_path, self.weight_file_name)
            self.model = models.load_model(self.model_path, backbone_name='resnet50')

        except :
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(self.model_path))

    def load_classes(self) :
        self.class_list = []
        
        with open(self.model_dir_path + "/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        
        return self.class_list

    def get_predictions(self, cv_image) :
        if cv_image is None:
            # TODO: show warning message (different color, maybe)
            return None,None

        else :                 
            
            # copy to draw on
            self.frame = cv_image.copy()
            # preprocess image for network
            input = preprocess_image(self.frame)
            input, scale = resize_image(input)
    
            self.frame_count += 1
            self.total_frames += 1
    
            # process image
            start = time.time()
            boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(input, axis=0))
            #print("processing time: ", time.time() - start)
    
            # correct for image scale
            boxes /= scale
    
            self.create_predictions_list(labels, scores, boxes)
    
            # visualize detections
            for box, score, label in zip(boxes[0], scores[0], labels[0]):
                # scores are sorted so we can break
                if score < self.conf_threshold:
                    break
                    
                color = label_color(label)
                
                b = box.astype(int)
                draw_box(self.frame, b, color=color)
                
                caption = "{} {:.3f}".format(self.labels_to_names[label], score)
                #print(self.labels_to_names[label])
                draw_caption(self.frame, b, caption)
    
            if self.show_fps :
                if self.frame_count >= 30:
                    self.end = time.time_ns()
                    self.fps = 1000000000 * self.frame_count / (self.end - self.start)
                    self.frame_count = 0
                    self.start = time.time_ns()
                
                if self.fps > 0:
                    self.fps_label = "FPS: %.2f" % self.fps
                    cv2.putText(self.frame, self.fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
            return (self.predictions, self.frame)
    


