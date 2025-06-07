import cv2
import numpy as np
import asyncio
import time
import RPi.GPIO as GPIO

from types import SimpleNamespace
from common.utils.json_utils import read_json
from common.utils.log import get_logger

from common.video.camera_coordinate_transformer import CameraCoordinateTransformer
from common.video.video_capture import VideoCapture

from common.video.distance_calculations import detect_marker_and_get_gps, cam_mat, dist_coef
from common.video.gps_pull_mod import gpsgrabber
from common.video.LORA_comms import send_rfm
from common.video.fps_counter import FPSTracker
from common.detectors.detector import Detector
from common.detectors.detector_manager import DetectorManager
from common.video.asyncfunc import CONCURRENT_FRAME_AND_GPS
from common.video.rpi_test import trigger_flags
from pymavlink import mavutil
from common.video.gps_pull_mod import arm_cont
from common.video.distance_calculations import pythag_distance

logger = get_logger(__name__)

distance = 0
marker_lat = 0
marker_long = 0

# Load calibration data
calib_data_path = r"MultiMatrix.npz"
calib_data = np.load(calib_data_path)
 

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

target_IDs = [5,  3]
marker_size_cm = 25.4


serial_port = '/dev/ttyACM0'
baudrate =  115200

correct =False
#print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(serial_port, baud=baudrate)

#print("Waiting for heartbeat...")
master.wait_heartbeat()


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


    def check_ids(self, t_frame, found_ids, t_lat, t_lon, t_hdg):
        """ Checks if ids matches target id"""
        if found_ids is None:
            return t_frame, None, t_lat, t_lon, t_hdg
        
        #Correct Marker Flag throws true if any ids are equivalent to target_IDs
        self.correct_marker = any(id in target_IDs for id in found_ids)
        
        if self.correct_marker:
            global correct
            correct = True
            #and not self.marker_detected_before:
            #self.datapack_save(t_frame, found_ids)
            light = trigger_flags(1,1)
            self.marker_detected_before = True
           
            

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

        #pix = arm_cont()
    
        #fix this to wait fopix = arm_cont()

        #while (pix == True):
        while True: 
            # get frame from the video capture 
            frame = self.video_capture._capture_frames()

            if frame is None:
                logger.warning("Recieved Empty Frame")
                cv2.waitKey(1)
             
            if frame is not None:
                # Grab lat,lon, hdg from pixhawk
                lat, lon, hdg = gpsgrabber()
                #logger.info(f"Received GPS Data - Lat: {lat}, Lon: {lon}, Heading: {hdg}")

                frame = self.video_capture._capture_frames()
                frame_procc = None
                
                corners, ids, _ = self.detector.detect(frame, True)

                
                # add a wait time 
                t_frame, matched_ids, t_lat, t_lon, t_hdg= self.check_ids(frame, ids, lat, lon, hdg)
                #logger.info(f"Matched id: {ids}")

                
                                
                #5 if we found a new “correct” marker, compute its GPS,
                #if matched_ids is not None:
                
                distance, frame_p= detect_marker_and_get_gps(
                    t_frame,
                    marker_size_cm,
                )

                if (correct == True):
                    asyncio.run(CONCURRENT_FRAME_AND_GPS(frame_p, ids))
                    
                if distance is not None:
                    marker_lat, marker_long = pythag_distance (distance, t_lat, t_lon, hdg)

                    logger.info(f"pythag calc: {marker_lat}, {marker_long}")
                    #logger.info(f"Marker GPS Coordinates OF correct marker→ lat: {marker_lat}, lon: {marker_long}")
                    # add a frame time of 5sec or so 
                else: 
                    continue

            
                cv2.imshow("Video Capture", frame_p)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                #marker_lat, marker_long = send_rfm(marker_lat, marker_long)
               
            '''
            # else: delete temp data

                    # if no corners are detected, show the frame and continue to next frame

            
            if not corners:
                if self.conf.video.show_video:
                    cv2.imshow("Video Capture", frame)
                    print("after video capture")
                    #wait for user to press any key
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break     continue
            '''
           
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.clean_up()
        logger.info("Shutting down gracefully")




if __name__ == "__main__":
    config: SimpleNamespace = read_json("config.json")

    auto_uav = AutoUav(config)
    auto_uav.run()
