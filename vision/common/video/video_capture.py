"""Module for video capture."""

import cv2
import numpy
import queue
import threading
from types import SimpleNamespace
from typing import Union

from common.utils.log import get_logger
from common.video.camera_calibrator import CameraCalibrator

logger = get_logger(__name__)


class VideoCapture:
    """Wrapper for the cv2.VideoCapture.

    This class implements a video capture in a separate thread. This allows the object detection
    to not be blocked by the video capture process, thus improving performance.

    Attr:
        video_cap: Wrapped cv2 VideoCapture object
        frame_buffer: Video frame buffer
        stop_event: Thread event to stop streaming
    """

    def __init__(self, video_conf: SimpleNamespace) -> None:
        """Initializes VideoCaptureThreaded.

        Args:
            video_conf: Video configuration

        Raises:
            RunTimeError: Failed to initialize Video Capture
        """
        self.video_cap: cv2.VideoCapture = cv2.VideoCapture(video_conf.source_idx)

        if self.video_cap is None or not self.video_cap.isOpened():
            raise RuntimeError("Failed to initialize Video Capture. Try a different index.")

        self.video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_conf.width)
        self.video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_conf.height)
        self.frame_buffer: queue.Queue = queue.Queue(maxsize=video_conf.max_buffer_size)
        self.stop_event: threading.Event = threading.Event()

    def start(self) -> None:
        """Starts the video capture thread."""
        logger.info("Starting video capture thread")
        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.capture_thread.start()

    def _capture_frames(self) -> None:
        """Capture frames from the camera and put it in the buffer."""
        while not self.stop_event.is_set():
            # ret: bool, frame: numpy.ndarray
            ret, frame = self.video_cap.read()
            if not ret:
                break
            # only populate to frame buffer if there is available space
            if not self.frame_buffer.full():
                self.frame_buffer.put(frame)
        self.video_cap.release()

    def stop(self) -> None:
        """Stop the video capture thread."""
        logger.info("Stopping video capture")
        self.stop_event.set()
        self.capture_thread.join()

    def read(self) -> Union[numpy.ndarray, None]:
        """Read a frame from the frame_buffer (not from the VideoCapture).

        Returns: A numpy.ndarray representing a frame or None if frame buffer is empty
        """
        if not self.frame_buffer.empty():
            return self.frame_buffer.get()
        return None
