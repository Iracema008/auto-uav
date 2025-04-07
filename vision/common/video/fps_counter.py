"""Module for FPS tracker."""

import cv2
import time
import numpy as np


class FPSTracker:
    """FPS Tracker.

    Attributes:
        last_time: The last time FPS was updated
        frame_count: Number of frames
        fps: Frames per second
    """

    def __init__(self) -> None:
        """Construct FPSTracker."""
        # use perf_counter for high precision
        self.last_time: float = time.perf_counter()
        self.frame_count: int = 0
        self.fps: float = 0

    def update(self) -> None:
        """Update FPS."""
        # calculate the time difference between the current and last frame
        current_time = time.perf_counter()
        time_diff = current_time - self.last_time
        self.frame_count += 1

        # update FPS every 10 frames (non-blocking)
        if self.frame_count >= 10:
            self.fps = self.frame_count / time_diff
            self.last_time = current_time  # Reset the timer
            self.frame_count = 0  # Reset the frame count

    def put_fps_on_frame(self, frame: np.ndarray) -> None:
        """Puts the FPS on the screen top left.

        Args:
            frame: np.ndarray that represents the video frame
        """
        cv2.putText(
            frame, f"FPS: {self.fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2
        )

    def get_fps(self) -> float:
        """Get current FPS.

        Returns:
            The current FPS
        """
        return self.fps
