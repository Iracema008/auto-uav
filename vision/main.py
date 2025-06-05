import cv2
import numpy as np
import asyncio

from types import SimpleNamespace
# from common.video.fps_counter import FPSTracker
from common.utils.json_utils import read_json
from common.utils.log import get_logger
from common.video.camera_coordinate_transformer import CameraCoordinateTransformer
from common.video.video_capture import VideoCapture

from common.video.distance_calculations import detect_marker_and_get_gps, cam_mat, dist_coef

from common.video.gps_pull_mod import gpsgrabber
# from common.video.asyncfunc import  CONCURRENT_FRAME_AND_GPS

from common.video.LORA_comms import send_rfm
from common.video.fps_counter import FPSTracker
from common.detectors.detector import Detector
from common.detectors.detector_manager import DetectorManager
from common.video.asyncfunc import CONCURRENT_FRAME_AND_GPS

logger = get_logger(__name__)

distance = 0
marker_lat = 0
marker_long = 0

# Load calibration data
calib_data_path = r"MultiMatrix.npz"
calib_data = np.load(calib_data_path)


cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

target_IDs = [2, 7 ]
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
       
        self.detector: Detector = DetectorManager(conf.detector).get_detector()
        self.fps_tracker: FPSTracker = FPSTracker()

        self.use_depthai = getattr(conf, "use_depthai", False)
        self.correct_marker = False
        self.marker_detected_before =False

    # assuming async def is here to save all frame dats
    async def temp_save(self, frame, lat=None, lon=None, hdg=None) -> None:
        pass

    # we then want to check id to process the temp saved to deicede wheter to return
    # should read temp data and return perm
    def check_ids(self, t_frame, found_ids, t_lat, t_lon, t_hdg):
        # run on all temp saved
        if found_ids is None:
            # print("no ids")
            return t_frame, None, t_lat, t_lon, t_hdg
        
        #Correct Marker Flag throws true if any ids are equivalent to target_IDs
        #loop that checks if anything in goal_ids is in target -- any(TRUE) flags true on any response
        self.correct_marker = any(id in target_IDs for id in found_ids)

        #return correct_marker
        #If the marker is true, we save the datapack
        if self.correct_marker:
            #and not self.marker_detected_before:
            #self.datapack_save(t_frame, found_ids)
            self.marker_detected_before = True
            # possibly add save CORRECT MARKER, instead of just temp
            return t_frame, found_ids, t_lat, t_lon, t_hdg

        # If no correct_marker but found_ids wasn’t None, still return something
        return t_frame, None, t_lat, t_lon, t_hdg
    
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
                # print("Got GPS Coordinates and Frame at same time.")
                #= asyncio.run(CONCURRENT_FRAME_AND_GPS(frame, ids))
                
                # add a wait time 
                #print("about to save permant_data")
                t_frame, matched_ids, t_lat, t_lon, t_hdg= self.check_ids(frame, ids, lat, lon, hdg)
                print("perm data after check ids")

                #5) If we found a new “correct” marker, compute its GPS,
                if matched_ids is not None:
                    marker_lat, marker_long = detect_marker_and_get_gps(
                        t_frame,
                        marker_size_cm,
                        cam_mat,
                        dist_coef,
                        t_lat,
                        t_lon,
                        t_hdg,
                    )
                    send_rfm(marker_lat, marker_long)
                    print(f"Marker GPS Coordinates → lat: {marker_lat}, lon: {marker_long}")
            
                #if permanent_data is not None:
                # add to check correct marker and then save as perm, send that to calculator distance
                # print(f"Permanent Data Saved: {permanent_data}")

                # function atrs of frame
                print("function attributes ")
                
                marker_size_cm = 30.48

                cam_mat = calib_data["camMatrix"]
                dist_coef = calib_data["distCoef"]
                # marker_size_cm = permanent_data.get("marker_size_cm", 10)
                # cam_mat = permanent_data.get("cam_mat")
                # dist_coef = permanent_data.get("dist_coef")
                # drone_lat = permanent_data.get("lat")
                # drone_long = permanent_data.get("lon")
                # heading_deg = permanent_data.get("hdg")

                # call the dist_calc function

                #This is not NONE, should be fixed
                #am_mat = None
                #ist_coef = None
                frame, marker_size_cm, cam_mat, dist_coef, drone_lat, drone_long, heading_deg = detect_marker_and_get_gps(
                    frame, marker_size_cm, cam_mat, dist_coef, drone_lat, drone_long, heading_deg
                )
                # call lora comms
                send_rfm(marker_lat, marker_long)

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
