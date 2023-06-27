
import cv2
import tensorflow as tf
import os
import numpy as np

# For measuring the inference time.
import time 

class EfficientDet:
    def __init__(self, model_dir_path, weight_file_name, conf_threshold = 0.35, score_threshold = 0.25, nms_threshold = 0.4):
        
        self.model_dir_path = model_dir_path
        self.weight_file_name = weight_file_name        

        self.conf = conf_threshold
        
        self.img_height = 640
        self.img_width = 640
        self.predictions = []
        self.bb_colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

        self.build_model()
        self.load_classes()

    def build_model(self) :
        model_path = os.path.join(self.model_dir_path, self.weight_file_name)
        try :
            self.detector = tf.saved_model.load(model_path)
        except :
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(model_path))
        
    def load_classes(self):
        self.labels = []
        with open(self.model_dir_path + "/classes.txt", "r") as f:
            self.labels = [cname.strip() for cname in f.readlines()]
        return self.labels

    def rescale_box(self,pred_boxes):
        rescaled_boxes = []

        # rescale boxes
        for box in pred_boxes:
            ymin, xmin, ymax, xmax = tuple(box)
            (left, right, top, bottom) = (xmin * self.image_width, xmax * self.image_width,
                                            ymin * self.image_height, ymax * self.image_height)
            rescaled_box = np.array([left,top,right,bottom]).astype(int)
            rescaled_boxes.append(rescaled_box)    

        return rescaled_boxes

    def get_predictions(self,cv_image):

        #Convert img to RGB
        frame = cv_image.copy()

        # Image dimensions
        self.image_height, self.image_width, _ = frame.shape

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # COnverting to uint8
        rgb_tensor = tf.convert_to_tensor(rgb, dtype=tf.uint8)

        #Add dims to rgb_tensor
        rgb_tensor = tf.expand_dims(rgb_tensor , 0)

        start_time = time.time()
        result = self.detector(rgb_tensor)
        end_time = time.time()

        # refactor resutls
        result = {key:value.numpy() for key,value in result.items()}
        result["detection_boxes"][0] = self.rescale_box(result["detection_boxes"][0])
        result["detection_classes"][0] = result["detection_classes"][0] -1

        print("Found %d objects." % len(result["detection_scores"][0]))
        print("Inference time: ", end_time-start_time)
        
        self.create_predictions_list(result["detection_classes"][0], result["detection_scores"][0], result["detection_boxes"][0].astype(int))

        # draw bounding box and add label
        
        # for (classid, confidence, box) in zip(result["detection_classes"][0], result["detection_scores"][0], result["detection_boxes"][0].astype(int)):
        #     if confidence >= self.conf:
        #         color = self.bb_colors[int(classid) % len(self.bb_colors)]
        #         cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
        #         cv2.putText(frame, f"{self.labels[int(classid)]} {confidence:.2f}", (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)

        return self.predictions