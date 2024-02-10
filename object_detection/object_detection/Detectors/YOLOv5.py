# Copyright (c) 2023 A.T.O.M ROBOTICS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import torch
import os

from ..DetectorBase import DetectorBase


class YOLOv5(DetectorBase):
    def __init__(self, conf_threshold = 0.7):

        super().__init__()

        self.conf_threshold = conf_threshold

    def build_model(self, model_dir_path, weight_file_name):

        try : 
            model_path = os.path.join(model_dir_path, weight_file_name)
            self.model = torch.hub.load('ultralytics/yolov5:v6.0', 'custom', path = model_path, force_reload = True)

        except :
            raise Exception("Error loading given model from path: {}. Maybe the file doesn't exist?".format(model_path))
        
    def load_classes(self, model_dir_path):

        self.class_list = []

        with open(os.path.join(model_dir_path, 'classes.txt')) as f :
            self.class_list = [cname.strip() for cname in f.readlines()]

        return self.class_list
    
    def get_predictions(self, cv_image):

        if cv_image is None :
            # TODO: show warning message (different color, maybe)
            return None, None
        
        else :
            self.frame = cv_image
            class_id = []
            confidence = []
            boxes = []

            results = self.model(self.frame)
            
            for *xyxy, conf, id in results.xyxy[0]:
                class_id.append(int(id))
                confidence.append(conf.item())
                boxes.append([int(xy) for xy in xyxy])

            super().create_predictions_list(class_id, confidence, boxes)

            return self.predictions
