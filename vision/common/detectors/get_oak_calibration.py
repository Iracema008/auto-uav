# Filename: get_oak_calibration.py
# (This script should be correct if run properly)

import depthai as dai
import numpy as np
import pickle
import os
import sys

# --- Configuration ---
OUTPUT_DIR = "aruco_dir"
OUTPUT_FILENAME = "datacalib_mtx_webcam.pkl"
OUTPUT_PATH = os.path.join(OUTPUT_DIR, OUTPUT_FILENAME)
TARGET_SOCKET = dai.CameraBoardSocket.RGB # Or try dai.CameraBoardSocket.CAM_A
TARGET_WIDTH = 0
TARGET_HEIGHT = 0

print("-" * 30)
print("Attempting to read calibration from OAK device...")
print(f"Target Camera Socket: {TARGET_SOCKET}")
print(f"Target Resolution (0=native): {TARGET_WIDTH}x{TARGET_HEIGHT}")
print(f"Output File: {os.path.abspath(OUTPUT_PATH)}")
print("-" * 30)

try:
    with dai.Device() as device:
        print("Device found...")
        calibData = device.readCalibration()

        if not calibData.eepromToJson():
             print("[ERROR] No calibration data found on device EEPROM.")
             sys.exit(1)

        print("Successfully read calibration data from EEPROM.")

        intrinsic_matrix = calibData.getCameraIntrinsics(
            TARGET_SOCKET,
            TARGET_WIDTH,
            TARGET_HEIGHT
        )

        if intrinsic_matrix is None or len(intrinsic_matrix) == 0:
            print(f"[ERROR] Could not retrieve intrinsic matrix for socket {TARGET_SOCKET}.")
            print("Possible reasons:")
            print("- Incorrect socket specified (is the camera RGB? Try CAM_A?).")
            print("- Calibration data might be missing for this specific camera on the EEPROM.")
            sys.exit(1)

        intrinsic_matrix = np.array(intrinsic_matrix)

        print(f"\nIntrinsic Matrix (fx, fy, cx, cy for {TARGET_SOCKET}):")
        print(intrinsic_matrix)

        try:
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            print(f"\nEnsured output directory exists: {OUTPUT_DIR}")

            # This is the correct way to save the pickle file
            with open(OUTPUT_PATH, 'wb') as f: # 'wb' is crucial
                pickle.dump(intrinsic_matrix, f)

            print(f"\n[SUCCESS] Successfully saved intrinsic matrix to: {OUTPUT_PATH}")
            print("-" * 30)

        except IOError as e:
            print(f"\n[ERROR] Error saving calibration file to {OUTPUT_PATH}: {e}")
        except Exception as e:
            print(f"\n[ERROR] An unexpected error occurred during saving: {e}")

except RuntimeError as e:
    print(f"\n[ERROR] Could not connect to OAK device or read calibration: {e}")
    print("Make sure the OAK camera is connected and no other application is using it.")
    print("On Linux, you might need to install udev rules: https://docs.luxonis.com/projects/api/en/latest/install/#install-udev-rules")
except Exception as e:
     print(f"\n[ERROR] An unexpected error occurred: {e}")
