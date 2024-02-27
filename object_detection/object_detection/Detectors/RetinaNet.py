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

import os

from keras_retinanet import models
from keras_retinanet.utils.image import preprocess_image, resize_image
import numpy as np

from ..DetectorBase import DetectorBase


class RetinaNet(DetectorBase):

    def __init__(self, logger):
        super().__init__(logger)

        # Create a logger instance
        self.logger = super().get_logger()

    def build_model(self, model_dir_path, weight_file_name):
        model_path = os.path.join(model_dir_path, weight_file_name)

        try:
            self.model = models.load_model(model_path, backbone_name='resnet50')
            self.logger.info("[RetinaNet] Model successfully loaded from: {}".format(model_path))
        except Exception as e:
            self.logger.error("[RetinaNet] Loading model failed with exception: {}".format(e))
            raise Exception("Error loading given model from path: {}.".format(model_path) +
                            " Maybe the file doesn't exist?")

    def load_classes(self, model_dir_path):
        self.class_list = []
        fpath = os.path.join(model_dir_path, 'classes.txt')

        try:
            with open(os.path.join(model_dir_path, "classes.txt"), "r") as f:
                self.class_list = [cname.strip() for cname in f.readlines()]
            self.logger.info("[RetinaNet] Loaded classes from {}".format(fpath))
        except FileNotFoundError:
            self.logger.error("[RetinaNet] Classes file not found at path: {}".format(fpath))
            raise FileNotFoundError("Classes file not found. Make sure the file exists at the specified path.")
        except Exception as e:
            self.logger.error("[RetinaNet] Error loading classes with exception: {}".format(e))
            raise Exception("Error loading classes from file: {}".format(fpath))

        return self.class_list

    def get_predictions(self, cv_image):
        if cv_image is None:
            self.logger.warning("[RetinaNet] Input image is None. No predictions will be generated.")
            return None
        else:
            # copy to draw on
            self.frame = cv_image.copy()
            # preprocess image for network
            processed_img = preprocess_image(self.frame)
            processed_img, scale = resize_image(processed_img)

            # process image
            boxes_all, confidences_all, class_ids_all = self.model.predict_on_batch(np.expand_dims(processed_img, axis=0))

            boxes, confidences, class_ids = [], [], []

            for index in range(len(confidences_all[0])):
                if confidences_all[0][index] != -1:
                    confidences.append(confidences_all[0][index])
                    boxes.append(boxes_all[0][index])
                    class_ids.append(class_ids_all[0][index])

            # correct for image scale
            # boxes = [x/scale for x in boxes]
            boxes = [[int(coord/scale) for coord in box] for box in boxes]

            super().create_predictions_list(class_ids, confidences, boxes)
            self.logger.debug("[RetinaNet] Object detection successfully performed on the input image.")

            return self.predictions
