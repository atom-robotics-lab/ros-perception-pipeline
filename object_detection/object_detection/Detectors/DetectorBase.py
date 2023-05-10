from abc import ABC, abstractmethod
import numpy as np


class DetectorBase(ABC):

    @abstractmethod
    def get_predictions(self, cv_image: np.ndarray) -> tuple[dict(), np.ndarray]:
        pass