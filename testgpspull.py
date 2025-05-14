# Import the MAVLink setup
from pymavlink import mavutil

# Replace with your serial port and baudrate (Pixhawk default is 57600)
serial_port = '/dev/ttyAMA0'
baudrate = 57600

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(serial_port, baud=baudrate)

# Wait for the first heartbeat to confirm communication
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from system (ID {master.target_system}, component {master.target_component})")



'''

# Listen for GPS data
print("Listening for GPS_RAW_INT messages...")
while True:
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # altitude in meters
        print(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}, Altitude: {alt:.2f} m")
'''

# 3) Loop, grabbing GPS_RAW_INT messages
print("Waiting for GPS_RAW_INT packets…\n(Press Ctrl‐C to exit)\n")
#print("trest")


#print(msg)
#print("test")


'''

    function to pull gps
'''

while True:
     # blocking=True will wait until a GPS_RAW_INT is received
    print("testddd")
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout = 5)

    #print(master.recv_match(type='GPS_RAW_INT', blocking=True)


    print(msg)

    if not msg:
        continue

    # msg.lat and msg.lon are int32 in 1E-7 degrees
    lat = msg.lat * 1e-7
    lon = msg.lon * 1e-7
    # msg.alt is int32 in millimetres above mean sea level
    alt = msg.alt * 1e-3

    fix_type = msg.fix_type    # 0-1 = no fix, 2 = 2D, 3 = 3D
    sats     = msg.satellites_visible

    print(f"[Fix {fix_type}, Sats {sats:2d}] "
          f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f} m")
    # optional: time.sleep(1)  # pacing, if you like
