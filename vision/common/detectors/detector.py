"""Abstract class for ArUco detectors."""

import numpy as np
from abc import ABC
from types import SimpleNamespace


class Detector(ABC):
    """This class serves as the abstract base class for all ArUco detectors.

    Attr:
        detector_conf: Configuration for the detector
    """

    def __init__(self, **kwargs) -> None:
        """Constructor for Detector"""
        pass

    def detect(self, frame: np.ndarray, **kwargs) -> None:
        """Abstract method to detect arUco markers.

        Args:
            frame: np.ndarray representing a video frame
            kwargs: additional args
        """
        pass

    def draw_detections(self, frame: np.ndarray, **kwargs) -> None:
        """Abstract method to draw the detection bounding boxes.

        Args:
            frame: np.ndarray representing a video frame
            kwargs: additional args
        """
        pass
