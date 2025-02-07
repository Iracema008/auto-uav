"""Module for the detector manager."""

from types import SimpleNamespace

from common.detectors.detector import Detector
from common.detectors.available_detectors import AvailableDetectors
from common.utils.log import get_logger

logger = get_logger(__name__)


class DetectorManager:
    """Class that manages detectors."""

    def __init__(self, detector_conf: SimpleNamespace) -> None:
        """Constructor for DetectorManager.

        Args:
            detector_conf: Detector configurations
        """
        self.detector_conf = detector_conf

    def get_detector(self) -> Detector:
        """Retrieves the configured detector.

        Returns:
            The configured detector

        Raises:
            KeyError: Configured detector type does not exist
        """
        detector_type = self.detector_conf.detector_type
        try:
            return AvailableDetectors[detector_type].value()
        except KeyError as e:
            logger.error(f"Detector type: {detector_type} does not exist. {e}")
            raise
