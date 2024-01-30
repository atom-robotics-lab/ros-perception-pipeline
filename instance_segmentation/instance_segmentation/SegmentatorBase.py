from abc import ABC, abstractmethod
import numpy as np


class SegmentatorBase(ABC):

    def __init__(self) -> None:
        self.predictions = []

    def create_predictions_list(self, class_ids, confidences, masks):
        for i in range(len(class_ids)):
            obj_dict = {
                "class_id": class_ids[i],
                "confidence": confidences[i],
                "mask": masks[i]
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
    
    @abstractmethod
    def get_segmented_image(self, cv_image: np.ndarray) -> list[dict]:
        pass