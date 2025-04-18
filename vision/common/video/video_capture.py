"""Module for video capture."""

import cv2
import depthai as dai
import numpy
import queue
import threading
import time

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
        

        # TODO: Integrate depthai frame capture with current opencv implementation
        self.use_depthai = getattr(video_conf, "use_depthai", False)


        if self.use_depthai:
            # Create pipeline (graph of nodes)
            pipeline = dai.Pipeline()

            # Creating camera nodes
            camera_rgb = pipeline.create(dai.node.ColorCamera)
            
            camera_rgb.setPreviewSize(640,480)
            camera_rgb.setInterleaved(False)
            camera_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            x_out = pipeline.create(dai.node.XLinkOut)

            x_out.setStreamName("rgb")
            # Linking output and input stream
            camera_rgb.preview.link(x_out.input)
        
            self.device = dai.Device(pipeline) 
            # Output queue will be used to get the rgb frames from the output defined above
            self.queue_rgb = self.device.getOutputQueue("rgb", maxSize =10 , blocking= True)
            #logger.info("Initialized Depthai pipeline")
        else:
            self.video_cap: cv2.VideoCapture = cv2.VideoCapture(0)    
         
            if self.video_cap is None or not self.video_cap.isOpened():
                raise RuntimeError("Failed to initialize Video Capture. Try a different index.")
            
            self.video_cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_conf.width)
            self.video_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_conf.height)

        self.frame_buffer: queue.Queue = queue.Queue(maxsize=video_conf.max_buffer_size)
        self.stop_event: threading.Event = threading.Event()

    def start(self) -> None:
        """Starts the video capture thread."""
        logger.info("Starting video capture thread")
        try: 
            self.capture_thread = threading.Thread(target=self._capture_frames)
            self.capture_thread.start()
            logger.info("Threading starts successfully")
        except Exception as e:
            logger.error(f"Error starting thread: {e}")

    def _capture_frames(self) -> None:
        """Capture frames from the camera and put it in the buffer."""
        self.saved_once = False  # Only save one frame per run

        while not self.stop_event.is_set():
            if self.use_depthai:
                while True:
                    try:
                        # Get frame from the DepthAI queue
                        input_rgb = self.queue_rgb.get()
                        frame_rgb = input_rgb.getCvFrame()

                        if not self.frame_buffer.full():
                            self.frame_buffer.put(frame_rgb)
                        '''
                        if frame_rgb is not None:
                            # Save the frame only once
                            if not self.saved_once:
                                cv2.imwrite("captured_frame.png", frame_rgb)
                                print("framed saved, captured frames function")
                                self.saved_once = True

                            # Optional logging
                        if not self.frame_buffer.full():
                            logger.warning("Buffer not full")
                            self.frame_buffer.put(frame_rgb)
                            # logger.debug("Frame added to buffer")
                        '''
                        return self.frame_buffer.get()     
                    except queue.Empty:
                        print("waiting for frame from dephai")
                        continue
                 
            '''else:
                # ret: bool, frame: numpy.ndarray
                # read function is returning image into"frame" 
                # if no frames are grabbed, will be empty 
                ret, frame = self.video_cap.read()
                if not ret:
                    logger.error("Failed to read frame from capture")
                    #break
                # only populate to frame buffer if there is available space
                if not self.frame_buffer.full():
                    logger.warning("spit out frame")
                    self.frame_buffer.put(frame)
                
                self.video_cap.release()
            '''

    def stop(self) -> None:
        """Stop the video capture thread."""
        logger.info("Stopping video capture")
        self.stop_event.set()
        self.capture_thread.join()

    def read(self) -> Union[numpy.ndarray, None]:
        """Read a frame from the frame_buffer (not from the VideoCapture).

        Returns: A numpy.ndarray representing a frame or None if frame buffer is empty
        """ 
        
        #print(self.frame_buffer, "crying")
        #print(self.frame_buffer.get())
        if not self.frame_buffer.empty():
            #print(self.frame_buffer.qsize())
            #frame =  self.frame_buffer.get()
            #cv2.imwrite("frame_buffer.png", frame)
            #cv2.imshow(self.frame_buffer.get())
            return self.frame_buffer.get()
        #return None
        
