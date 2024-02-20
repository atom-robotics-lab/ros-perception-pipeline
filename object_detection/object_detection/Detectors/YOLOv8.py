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

from ultralytics import YOLO

from ..DetectorBase import DetectorBase


class YOLOv8(DetectorBase):

    def __init__(self, conf_threshold=0.7):
        super().__init__()
        self.conf_threshold = conf_threshold

    def build_model(self, model_dir_path, weight_file_name):
        try:
            model_path = os.path.join(model_dir_path, weight_file_name)
            self.model = YOLO(model_path)
        except Exception as e:
            print("Loading model failed with exception: {}".format(e))
            raise Exception("Error loading given model from path: {}.".format(model_path) +
                            " Maybe the file doesn't exist?")

    def load_classes(self, model_dir_path):
        self.class_list = []

        with open(model_dir_path + "/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]

        return self.class_list

    def get_predictions(self, cv_image):
        if cv_image is None:
            # TODO: show warning message (different color, maybe)
            return None, None
        else:
            self.frame = cv_image
            class_id = []
            confidence = []
            boxes = []

            # Perform object detection on image
            result = self.model.predict(self.frame, conf=self.conf_threshold, verbose=False)
            row = result[0].boxes.cpu()

            for box in row:
                class_id.append(box.cls.numpy().tolist()[0])
                confidence.append(box.conf.numpy().tolist()[0])
                boxes.append(box.xyxy.numpy().tolist()[0])

            super().create_predictions_list(class_id, confidence, boxes)

            return self.predictions
