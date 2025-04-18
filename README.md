# autonomous-uav

### Running the ArUco detection
1. Ensure git lfs is installed and initialized:

    https://git-lfs.com/
2. Clone the repository:

    `git clone https://github.com/Iracema008/auto-uav.git && cd auto-uav`
3. Change the working directory:

    `cd vision`
4. Run the command:

    `source helper.sh`
5. Run the main program:

    `run`


### Running the Simulator
#### Ensure that the following prerequisites are met:
1. Install ArduPilot:
    - [Set up your ArduPilot Build Environment](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
      - **NOTE**: Select the correct autopilot board when building with waf. We are using **PIXHAWK1**
2. QGroundControl is already downloaded to the repository.
#### To run the simulator:
1. Clone the repository: 

    `git clone https://github.com/Iracema008/auto-uav.git`

2. In the root directory, run the command:

    `sim_vehicle.py -v ArduCopter --out="localhost:14550" --out="localhost:14551"`
3. Run QGroundControl:

    `./QGroundControl.AppImage`

### Running the Camera
1. 
    `cd depthai-python/ examples`
2. 
    `python3 ColorCamera/rgb_preview.py`


### Troubleshooting

Please ensure that you grant camera permissions to the IDE you are using if you are running this on a Mac.
