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

    def __init__(self, logger):

        super().__init__(logger)

    def build_model(self, model_dir_path, weight_file_name):
        try:
            model_path = os.path.join(model_dir_path, weight_file_name)
            self.model = YOLO(model_path)
            self.logger.info("[YOLOv8] Model successfully loaded from: {}".format(model_path))
        except Exception as e:
            self.logger.error("[YOLOv8] Failed to load model with exception: {}".format(e))
            raise Exception("Error loading the given model from path: {}.".format(model_path) +
                            " Make sure the file exists and the format is correct.")

    def load_classes(self, model_dir_path):
        self.class_list = []
        fpath = os.path.join(model_dir_path, "classes.txt")

        try:
            with open(fpath, "r") as f:
                self.class_list = [cname.strip() for cname in f.readlines()]
                self.logger.info("[YOLOv8] Loaded classes from {}".format(fpath))
        except FileNotFoundError:
            self.logger.error("[YOLOv8] Classes file not found at path: {}".format(fpath))
            raise FileNotFoundError("Classes file not found. Make sure the file exists at the specified path.")
        except Exception as e:
            self.logger.error("[YOLOv8] Error loading classes with exception: {}".format(e))
            raise Exception("Error loading classes from file: {}".format(fpath))

        return self.class_list

    def get_predictions(self, cv_image):
        if cv_image is None:
            self.logger.warning("[YOLOv8] Input image is None. No predictions will be generated.")
            return None, None
        else:
            self.frame = cv_image
            class_id = []
            confidence = []
            boxes = []

            # Perform object detection on image
            try:
                result = self.model.predict(self.frame, verbose=False)
                row = result[0].boxes.cpu()

                for box in row:
                    class_id.append(box.cls.numpy().tolist()[0])
                    confidence.append(box.conf.numpy().tolist()[0])
                    boxes.append(box.xyxy.numpy().tolist()[0])

                super().create_predictions_list(class_id, confidence, boxes)
                self.logger.debug("[YOLOv8] Object detection successfully performed on the input image.")
            except Exception as e:
                self.logger.error("[YOLOv8] Object detection failed with exception: {}".format(e))
                raise Exception("Error performing object detection on the input image.")

            return self.predictions
