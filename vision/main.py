'''auto_uav.py switched to "main.py"'''
import cv2
import numpy as np


import asyncio
from types import SimpleNamespace


# from common.video.fps_counter import FPSTracker
from common.utils.json_utils import read_json
from common.utils.log import get_logger
from common.video.camera_coordinate_transformer import CameraCoordinateTransformer
from common.video.video_capture import VideoCapture

from common.video.distance_calculations import detect_marker_and_get_gps
from vision.common.video.gps_pull_mod import gpsgrabber

logger = get_logger(__name__)

distance = None,
marker_lat = None,
marker_long = None,

# Load calibration data

calib_data_path = "C:/home/pi/Downloads/MultiMatrix.npz"  # TODO: Update path for Raspberry Pi
calib_data = np.load(calib_data_path)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

target_IDs = [3, 7 ]
marker_size_cm = 30.48


class AutoUav:
    """Main autonomous UAV class.

    Attr:
        conf: App configurations
        video_capture: Video capture class that provides frames for object detection
        detector: Target identifier/detector
        fps_tracker: Tracks the fps of image processing
    """

    def __init__(self, conf: SimpleNamespace) -> None:
        """Constructor for AutoUav.

        Args:
            conf: App configuration
        """
        self.conf = conf
        self.video_capture: VideoCapture = VideoCapture(conf.video)
        self.camera_coordinate_transformer: CameraCoordinateTransformer = (
            CameraCoordinateTransformer(conf.video)
        )
        self.use_depthai = getattr(conf, "use_depthai", False)

    # assuming async def is here to save all frame dats
    async def temp_save(self, frame, FLIGHTDATA, lat=None, lon=None, hdg=None) -> None:
        pass

    # we then want to check id to process the temp saved to deicede wheter to return
    # should read temp data and return perm
    def check_ids(self, frame, found_ids, curr_lat, curr_lon, curr_hdg):
        # run on all temp saved
        if found_ids is None:
            print("no ids")
            return
        if any(id_ in target_IDs for id_ in found_ids):
            # will save temp if yes
            # self.temp_save(self, frame, found_ids)

            print("found target ids, now saving as permanent data.")
            return {"lat": curr_lat, "lon": curr_lon, "hdg": curr_hdg}

        return None
    
    def clean_up(self) -> None:
        """Cleanup for AutoUav."""
        logger.info("Cleaning up")
        self.video_capture.stop()
        cv2.destroyAllWindows()




    def run(self) -> None:
        """Runs the main logic."""
        logger.info("AutoUav starting up...")
        self.video_capture.start()


        while True:
            # get frame from the video capture 
            frame = self.video_capture._capture_frames()

            #drone_lat, drone_long, heading_deg = self.gpspull(),
            #lat, lon =  detect_marker_and_get_gps(frame, marker_size_cm, cam_mat, dist_coef, drone_lat, drone_long, heading_deg)
            # get frame from the video capture
            lat, lon, hdg = gpsgrabber()
            logger.info(f"Received GPS Data - Lat: {lat}, Lon: {lon}, Heading: {hdg}")

            frame = self.video_capture._capture_frames()

            if frame is None:
                logger.warning("Recieved Empty Frame")
                cv2.waitKey(1)
             
            if frame is not None:
            # detect aruco
                corners, ids, _ = self.detector.detect(frame, True)

                # assuming a call to async function here
                permanent_data = self.check_ids(frame, ids, lat, lon, hdg)
                # if we want to keep perm data
                if permanent_data:
                    # send perm data to calculator distance
                    # (frame, marker_size_cm, cam_mat, dist_coef, drone_lat, drone_long, heading_deg) ?
                    print(f"Permanent Data Saved: {permanent_data}")

                    # function atrs of frame
                    marker_size_cm = permanent_data.get("marker_size_cm", 10)
                    cam_mat = permanent_data.get("cam_mat")
                    dist_coef = permanent_data.get("dist_coef")
                    drone_lat = permanent_data.get("lat")
                    drone_long = permanent_data.get("lon")
                    heading_deg = permanent_data.get("hdg")

                    # call the dist_calc function
                    marker_lat, marker_long = detect_marker_and_get_gps(
                        frame, marker_size_cm, cam_mat, dist_coef, drone_lat, drone_long, heading_deg
                    )

                    print(f"Marker GPS Coordinates: Lat= {marker_lat}, Lon= {marker_long}")
                # else: delete temp data

            # if no corners are detected, show the frame and continue to next frame
            if not corners:
                if self.conf.video.show_video:
                    cv2.imshow("Video Capture", frame)
                    #wait for user to press any key
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue


            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.clean_up()
        logger.info("Shutting down gracefully")




if __name__ == "__main__":
    config: SimpleNamespace = read_json("config.json")

    auto_uav = AutoUav(config)
    auto_uav.run()
