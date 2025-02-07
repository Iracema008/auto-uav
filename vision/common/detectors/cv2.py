"""ArUco detector using cv2 built in detector."""

import cv2
import numpy as np

from common.detectors.detector import Detector
from common.utils.log import get_logger

logger = get_logger(__name__)


class Cv2Detector(Detector):
    """ArUco object detector.

    This detector is a wrapper for CV2 aruco marker detection. It does not use a vision model. It
    is implemented with pattern matching.

    Attributes:
        aruco_dict: Predefined dictionary of aruco markers
        parameters: Detection parameters
    """

    def __init__(self) -> None:
        """Constructor for Cv2Detector."""
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

    def detect(self, frame: np.ndarray, print_corners: bool = False) -> tuple:
        """Detect the arUco marker.

        This method is the concrete implementation of the parent class abstract method.

        Args:
            frame: np.ndarray representing the video frame
            print_corners: Flag to print corners or not.

        Returns: A tuple of the corners, ids, and rejected markers in the frame
        """
        # convert the image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect the markers in the frame
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if print_corners and corners:
            logger.debug(f"DETECTED ARUCO MARKER: \n\tcorner: {corners}\n\tid: {ids}")

        return corners, ids, rejected

    def draw_detections(self, frame: np.ndarray, corners: list, ids: list) -> np.ndarray:
        """Draws the detected marker's bounding boxes on the frame.

        Args:
            frame: np.ndarray representing the video frame
            corners: The corners of the detected markers
            ids: The ids of the detected markers

        Returns:
            The frame with the marker's boudning boxes drawn
        """
        # if markers are detected, draw them on the frame
        if ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame
