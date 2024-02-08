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

import numpy as np

from abc import ABC, abstractmethod


class DetectorBase(ABC):

    def __init__(self) -> None:
        self.predictions = []

    def create_predictions_list(self, class_ids, confidences, boxes):
        self.predictions = []
        for i in range(len(class_ids)):
            obj_dict = {
                "class_id": class_ids[i],
                "confidence": confidences[i],
                "box": boxes[i]
            }

            self.predictions.append(obj_dict)

    @abstractmethod
    def build_model(self, model_dir_path: str, weight_file_name: str) -> None:
        pass

    @abstractmethod
    def load_classes(self, model_dir_path: str) -> None:
        pass

    @abstractmethod
    def get_predictions(self, cv_image: np.ndarray) -> list[dict]:
        pass
