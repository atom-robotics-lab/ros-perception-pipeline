import os

from keras_retinanet import models
from keras_retinanet.utils.image import preprocess_image, resize_image
import numpy as np

from ..DetectorBase import DetectorBase


class RetinaNet(DetectorBase) :
    def __init(self) :

        super.__init__()

    def build_model(self, model_dir_path, weight_file_name) :
        model_path = os.path.join(model_dir_path, weight_file_name)
        
        try:        
            self.model = models.load_model(model_path, backbone_name='resnet50')
        except:
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(model_path))

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
            boxes_all, confidences_all, class_ids_all = self.model.predict_on_batch(np.expand_dims(input, axis=0))

            boxes, confidences, class_ids = [], [], []
            
            for index in range(len(confidences_all[0])) :
                if confidences_all[0][index]!=-1 :
                    confidences.append(confidences_all[0][index])
                    boxes.append(boxes_all[0][index])
                    class_ids.append(class_ids_all[0][index])
            
    
            # correct for image scale
            #boxes = [x/scale for x in boxes]
            boxes = [[int(coord/scale) for coord in box] for box in boxes]
            
            super().create_predictions_list(class_ids, confidences, boxes)
            
            return self.predictions
    


