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

import torch

from ..DetectorBase import DetectorBase


class YOLOv5(DetectorBase):

    def __init__(self, conf_threshold=0.7):
        super().__init__()
        self.conf_threshold = conf_threshold

    def build_model(self, model_dir_path, weight_file_name):
        try:
            model_path = os.path.join(model_dir_path, weight_file_name)
            self.model = torch.hub.load('ultralytics/yolov5:v6.0', 'custom', path=model_path,
                                        force_reload=True)
            self.logger.info("[YOLOv5] Model successfully loaded from: {}".format(model_path))
        except Exception as e:
            self.logger.error("[YOLOv5] Loading model failed with exception: {}".format(e))
            raise Exception("Error loading given model from path: {}.".format(model_path) +
                            " Maybe the file doesn't exist?")

    def load_classes(self, model_dir_path):
        self.class_list = []
        fpath = os.path.join(model_dir_path, 'classes.txt')
        try:
            with open(fpath) as f:
                self.class_list = [cname.strip() for cname in f.readlines()]
            self.logger.info("[YOLOv5] Loaded classes from {}".format(fpath))
        except FileNotFoundError:
            self.logger.error("[YOLOv5] Classes file not found at path: {}".format(fpath))
            raise FileNotFoundError("Classes file not found. Make sure the file exists at the specified path.")
        except Exception as e:
            self.logger.error("[YOLOv5] Error loading classes with exception: {}".format(e))
            raise Exception("Error loading classes from file: {}".format(fpath))

        return self.class_list

    def get_predictions(self, cv_image):
        if cv_image is None:
            self.logger.warning("[YOLOv5] Input image is None. No predictions will be generated.")
            return None, None
        else:
            self.frame = cv_image
            class_id = []
            confidence = []
            boxes = []

            results = self.model(self.frame)

            for *xyxy, conf, label in results.xyxy[0]:
                class_id.append(int(label))
                confidence.append(conf.item())
                boxes.append([int(xy) for xy in xyxy])

            super().create_predictions_list(class_id, confidence, boxes)
            self.logger.debug("[YOLOv5] Object detection successfully performed on the input image.")

            return self.predictions
