"""Module for the CameraCalibrator."""

import os
import glob
from types import SimpleNamespace

import cv2 as cv
import numpy as np

from common.utils.log import get_logger

logger = get_logger(__name__)


# we have a 20ft distance camera zoom

class CameraCalibrator:
    """This class calibrates the camera given its intrinsic parameters.

    Attr:
        calibrate_conf: Calibration configurations
        img_path_list: List of calibration images paths

    """

    def __init__(self, calibrate_conf: SimpleNamespace) -> None:
        """Constructor for CameraCalibrator.

        Args:
            calibrate_conf: Calibration configurations
        """
        self.calibrate_conf = calibrate_conf

    def calibrate(self) -> None:
        """Calibrate camera."""
        # get calibration images
        img_path_list: list[str] = glob.glob(
            os.path.join(self.calibrate_conf.img.img_path, f"*{self.calibrate_conf.img.img_format}")
        )
        # create matrix of (n_rows * n_cols, 3) and populate with zeros
        world_pts_cur = np.zeros(
            (self.calibrate_conf.n_rows * self.calibrate_conf.n_cols, 3), np.float32
        )
        # our intrinsic camera matrix
        # [ 1739    0   727.5000]
        # [ 0     1739     543.5]
        # [ 0       0          1]


        # create a 3d mesh grid where each row represents a point in the world coordinate.
        # the first column represents the x-coordinate, second column represents the
        # y-coordinate, and the third column represents the z-coordinate which we will assume as 0
        world_pts_cur[:, :2] = np.mgrid[
            0 : self.calibrate_conf.n_rows, 0 : self.calibrate_conf.n_cols
        ].T.reshape(-1, 2)
        world_pts_list = []
        img_pts_list = []

        # create stop criteria for corner detection refinement
        stop_criteria: tuple[enumerate, int, float] = (
            cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        # find corners of the pattern
        for cur_img_path in img_path_list:
            # read image and grayscale
            img_bgr = cv.imread(cur_img_path)
            img_gray = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY)
            corners_found, corners = cv.findChessboardCorners(
                img_gray, (self.calibrate_conf.n_rows, self.calibrate_conf.n_cols), None
            )

            logger.debug(world_pts_cur)
            if corners_found:
                world_pts_list.append(world_pts_cur)
                # get refined corners
                corners_refined = cv.cornerSubPix(
                    img_gray, corners, (11, 11), (-1, -1), stop_criteria
                )
                img_pts_list.append(corners_refined)

                if self.calibrate_conf.show_pics:
                    cv.drawChessboardCorners(
                        img_bgr,
                        (self.calibrate_conf.n_rows, self.calibrate_conf.n_cols),
                        corners_refined,
                        corners_found,
                    )
                    cv.imshow("Chessboard", img_bgr)
                    cv.waitKey(500)

        cv.destroyAllWindows()

        # run calibration
        rep_error, cam_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(
            world_pts_list, img_pts_list, img_gray.shape[::-1], None, None
        )
        logger.deb

        np.saves(
            "data/calibration/calibration.npz",
            rep_error=rep_error,
            cam_matrix=cam_matrix,
            dist_coeffs=dist_coeffs,
            rvecs=rvecs,
            tvecs=tvecs,
        )
