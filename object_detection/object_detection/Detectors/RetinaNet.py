import os

from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
import cv2
import numpy as np

from ..DetectorBase import DetectorBase


class RetinaNet(DetectorBase) :
    def __init(self, conf_threshold = 0.7, score_threshold = 0.4, 
                nms_threshold = 0.25, is_cuda = 0, show_fps = 1) :

        super.__init__()

        self.conf_threshold = conf_threshold
        self.show_fps = show_fps
        self.is_cuda = is_cuda

    def build_model(self, model_dir_path, weight_file_name) :
        model_path = os.path.join(model_dir_path, weight_file_name)
        
        #try:
        print(model_path)
        self.model = models.load_model(model_path, backbone_name='resnet50')
        #except:
            #raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(self.model_path))

    def load_classes(self, model_dir_path) :
        self.class_list = []
        
        with open(model_dir_path + "/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        
        return self.class_list

    def get_predictions(self, cv_image) :
        if cv_image is None:
            # TODO: show warning message (different color, maybe)
            return None
        
        else :                
            
            # copy to draw on
            self.frame = cv_image.copy()
            # preprocess image for network
            input = preprocess_image(self.frame)
            input, scale = resize_image(input)
    
            # process image
            boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(input, axis=0))
    
            # correct for image scale
            boxes /= scale
    
            super().create_predictions_list(labels, scores, boxes)
      
            return self.predictions
    


