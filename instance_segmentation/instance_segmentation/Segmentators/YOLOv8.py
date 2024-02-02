import cv2
from ultralytics import YOLO
import os
import sys
import time
import numpy as np

from ..SegmentatorBase import SegmentatorBase


class YOLOv8(SegmentatorBase):
    def __init__(self, conf_threshold = 0.7, 
                             score_threshold = 0.4, nms_threshold = 0.25,
                             show_fps = 1, is_cuda = 0):
        
        super().__init__()

        
        self.conf_threshold = conf_threshold
        self.show_fps = show_fps
        self.is_cuda = is_cuda

        #FPS
        if self.show_fps :
            self.frame_count = 0
            self.total_frames = 0
            self.fps = -1
            self.start = time.time_ns()
            self.frame = None


        self.predictions = []
        self.build_model()
        self.load_classes()


    def build_model(self,model_dir_path,weight_file_name) :

        try :
            model_path = os.path.join(model_dir_path, weight_file_name)
            self.model = YOLO(model_path)

        except :
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(model_path))

    def load_classes(self, model_dir_path):

        self.class_list = []

        with open(model_dir_path + "/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]

        return self.class_list

    def get_predictions(self, cv_image):

        if cv_image is None:
            # TODO: show warning message (different color, maybe)
            return None,None
        
        else :         
            self.frame = cv_image     
            self.frame_count += 1
            self.total_frames += 1

            class_ids = []
            confidences = []
            masks = []
            result = self.model.predict(self.frame, conf = self.conf_threshold) # Perform object detection on image
            row = result[0].boxes

            for box in row:
                class_ids.append(box.cls)
                confidences.append(box.conf)
                # masks.append(mask.xyxy)
            for mask in result[0].masks:
                masks.append(mask.xy)


            print("frame_count : ", self.frame_count)

            if self.show_fps:
                if self.frame_count >= 30:
                    self.end = time.time_ns()
                    self.fps = 1000000000 * self.frame_count / (self.end - self.start)
                    self.frame_count = 0
                    self.start = time.time_ns()

                if self.fps > 0:
                    self.fps_label = "FPS: %.2f" % self.fps
                    cv2.putText(cv_image, self.fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            return self.predictions, cv_image
        

    def get_segmented_image(self, cv_image):
        # results = self.model.predict(cv_image, conf = self.conf_threshold, save=True) # Perform object detection on image
        # results = self.model.predict(cv_image, conf = 0.2, save=True) # Perform object detection on image
        results = self.model.predict(cv_image, conf = 0.2, save=True, show=True) # Perform object detection on image
        # result_image_array = np.array(result_image)
        # print(results[0].path)
        # print("\nhuhuhaha\n")
        # cv2.imshow(result_image)
        # return result_image_array
        result_image = cv2.imread("~/git/percep_ws/src/ros-perception-pipeline/instance_segmentation/runs/segment/predict/image0.jpg")
        return result_image
