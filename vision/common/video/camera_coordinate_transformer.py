"""This module is for the CameraCoordinateTransformer."""

import math
from types import SimpleNamespace

from common.utils.log import get_logger

logger = get_logger(__name__)


class CameraCoordinateTransformer:
    """This class transforms 2D camera pixel coordinates to real-world 3D relative coordinates."""

    def __init__(self, video_conf: SimpleNamespace):
        """Initializes CameraCoordinateTransformer.

        Args:
            video_conf: Video configuration
        """
        self.video_conf: SimpleNamespace = video_conf
        self.camera_center: tuple[float, float] = (
            video_conf.width / 2,
            video_conf.height / 2,
        )
        self.video_fov: tuple[float, float] = (video_conf.fov_h, video_conf.fov_v)
        self.pixels_per_fov_degree: tuple[float, float] = (
            (video_conf.width / video_conf.fov_h),
            (video_conf.height / video_conf.fov_v),
        )

    def transform(
        self,
        camera_coordinate: tuple[float, float],
        altitude: float,
    ) -> tuple[float, float, float]:
        """This method transforms a 2D camera coordinate to a 3D world coordinate.

        The transformation is done by:
        1. Calculate the number of pixels per FOV degree of the camera
        2. Find the degrees (theta) of the detected target from the camera center
        3. Determine the distance in METERS using tangeant (tan(theta) = distance / altitude)

        Args:
            camera_coordinate: 2d coordinate of the detected aruco marker
            altitude: Current altitude of the vehicle

        Returns:
            Tuple of the transformed coordinates (x, y, z)
        """
        # get x and y distances from camera center to the coordinate in pixels
        x_distance_px = camera_coordinate[0] - self.camera_center[0]
        y_distance_px = self.camera_center[1] - camera_coordinate[1]

        # convert distances into angles in radians
        x_distance_angle = math.radians(x_distance_px / self.pixels_per_fov_degree[0])
        y_distance_angle = math.radians(y_distance_px / self.pixels_per_fov_degree[1])

        # using the angles and the altitude, estimate the x and y coordinates in meters
        x_distance_m = math.tan(x_distance_angle) * altitude
        y_distance_m = math.tan(y_distance_angle) * altitude
        z_distance_m = 0 - altitude

        coordinate_3d = (x_distance_m, y_distance_m, z_distance_m)

        logger.debug(f"2D COORDINATE: {camera_coordinate}, 3D COORDINATE: {coordinate_3d}")

        return coordinate_3d
