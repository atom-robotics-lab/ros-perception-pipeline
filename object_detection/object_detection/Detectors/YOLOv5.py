import os

import cv2
import numpy as np

from ..DetectorBase import DetectorBase


class YOLOv5(DetectorBase):
    def __init__(self, conf_threshold = 0.7, score_threshold = 0.4, nms_threshold = 0.25, is_cuda = 0):

        super().__init__()

        # opencv img input
        self.frame = None
        self.net = None
        self.INPUT_WIDTH = 640
        self.INPUT_HEIGHT = 640
        self.CONFIDENCE_THRESHOLD = conf_threshold

        self.is_cuda = is_cuda   

    
    # load model and prepare its backend to either run on GPU or CPU, see if it can be added in constructor
    def build_model(self, model_dir_path, weight_file_name):
        model_path = os.path.join(model_dir_path, weight_file_name)

        try:
            self.net = cv2.dnn.readNet(model_path)
        except:
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(model_path))

        if self.is_cuda:
            print("is_cuda was set to True. Attempting to use CUDA")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("is_cuda was set to False. Running on CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)


    # load classes.txt that contains mapping of model with labels
    # TODO: add try/except to raise exception that tells the use to check the name if it is classes.txt
    def load_classes(self, model_dir_path):
        self.class_list = []
        with open(model_dir_path + "/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        return self.class_list

    
    def detect(self, image):
        # convert image to 640x640 
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.INPUT_WIDTH, self.INPUT_HEIGHT), swapRB=True, crop=False)
        self.net.setInput(blob)
        preds = self.net.forward()
        return preds


    # extract bounding box, class IDs and confidences of detected objects
    # YOLOv5 returns a 3D tensor of dimension 25200*(5 + n_classes)
    def wrap_detection(self, input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.INPUT_WIDTH
        y_factor =  image_height / self.INPUT_HEIGHT

        # Iterate through all the 25200 vectors
        for r in range(rows):
            row = output_data[r]

            # Continue only if Pc > conf_threshold
            confidence = row[4]
            if confidence >= self.CONFIDENCE_THRESHOLD:
                
                # One-hot encoded vector representing class of object
                classes_scores = row[5:]

                # Returns min and max values in a array alongwith their indices
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)

                # Extract the column index of the maximum values in classes_scores
                class_id = max_indx[1]

                # Continue of the class score is greater than a threshold
                # class_score represents the probability of an object belonging to that class
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        # removing intersecting bounding boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes


    # makes image square with dimension max(h, w)
    def format_yolov5(self):
        row, col, _ = self.frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = self.frame
        return result


    def get_predictions(self, cv_image):
        
        if cv_image is None:
            # TODO: show warning message (different color, maybe)
            return None,None
        else:
            self.frame = cv_image

            # make image square
            inputImage = self.format_yolov5()

            outs = self.detect(inputImage)
            class_ids, confidences, boxes = self.wrap_detection(inputImage, outs[0])

            super().create_predictions_list(class_ids, confidences, boxes)
                            
            print("Detected ids: ", [self.class_list[id] for id in class_ids])         
            
            return self.predictions