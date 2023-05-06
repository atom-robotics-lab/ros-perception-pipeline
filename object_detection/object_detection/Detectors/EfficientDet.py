import tensorflow_hub as hub
import cv2
import numpy
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt

import tempfile


# For drawing onto the image.
import numpy as np
from PIL import Image
from PIL import ImageColor
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageOps

# For measuring the inference time.
import time 

class EfficientDet:
    def __init__(self, model_dir_path, weight_file_name, conf_threshold = 0.7, score_threshold = 0.25, nms_threshold = 0.4):
        
        self.model_dir_path = model_dir_path
        self.weight_file_name = weight_file_name

        self.conf=conf_threshold
        
        self.img_height=800
        self.img_width=800
        self.predictions=[]
        self.bb_colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]
        

        self.build_model()
        self.load_classes()

    def build_model(self) :
        module_handle="https://tfhub.dev/tensorflow/efficientdet/d0/1"
        # Loading model directly from TensorFlow Hub
        self.detector = hub.load(module_handle)

    def load_classes(self):
        self.labels = []
        with open(self.model_dir_path + "/classes.txt", "r") as f:
            self.labels = [cname.strip() for cname in f.readlines()]
        return self.labels

   
    # create list of dictionary containing predictions
    def create_predictions_list(self, class_ids, confidences, boxes):
        for i in range(len(class_ids)):
            obj_dict = {
                "class_id": class_ids[i],
                "confidence": confidences[i],
                "box": boxes[i]
            }
            self.predictions.append(obj_dict)



    def get_predictions(self,cv_image):

        #Convert img to RGB
        frame = cv_image.copy()

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # COnverting to uint8
        rgb_tensor = tf.convert_to_tensor(rgb, dtype=tf.uint8)

        #Add dims to rgb_tensor
        rgb_tensor = tf.expand_dims(rgb_tensor , 0)

        start_time = time.time()
        result = self.detector(rgb_tensor)
        end_time = time.time()

        result = {key:value.numpy() for key,value in result.items()}
        
        print("Found %d objects." % len(result["detection_scores"]))
        print("Inference time: ", end_time-start_time)
        
        self.create_predictions_list(result["detection_boxes"][0],result["detection_classes"][0], result["detection_scores"][0])
        
        # draw bounding box and add label
        for (classid, confidence, box) in zip(result["detection_classes"][0], result["detection_scores"][0], result["detection_boxes"][0]):
            color = self.bb_colors[int(classid) % len(self.bb_colors)]
            cv2.rectangle(frame, box, color, 2)
            cv2.rectangle(frame, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            try :
                cv2.putText(frame, self.labels[classid]+" "+confidence, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
            except :
                pass
        
        return self.predictions, frame


if __name__=='__main__':
  # Load model 
    det = EfficientDet()