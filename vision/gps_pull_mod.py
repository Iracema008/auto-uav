import time
import busio
import board

from pymavlink import mavutil

# Replace with your serial port and baudrate (Pixhawk default is 57600)
serial_port = '/dev/ttyACM0'
baudrate =  115200

#print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(serial_port, baud=baudrate)

#print("Waiting for heartbeat...")
master.wait_heartbeat()

master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    5,  # Hz
    1   # start
)


def gpsgrabber():
    # blocking=True will wait until a GPS_RAW_INT is received

    #msg = master.recv_msg()
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)

    print(msg.hdg_acc)

    #Course Over Ground - NOT HEADING
    #cog = msg.cog
    #Add heading
    hdg = msg.hdg_acc
    # msg.lat and msg.lon are int32 in 1E-7 degrees
    lat = msg.lat * 1e-7
    lon = msg.lon * 1e-7
    # msg.alt is int32 in millimetres above mean sea level
    alt = msg.alt * 1e-3

    fix_type = msg.fix_type    # 0-1 = no fix, 2 = 2D, 3 = 3D
    sats     = msg.satellites_visible

    #print(f"[Fix {fix_type}, Sats {sats:2d}] "
    #     f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f}, Heading: {hdg:.5f}, Course: {cog}")
    
    print(f"[Fix {fix_type}, Sats {sats:2d}] "
        f"Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f}, Heading: {hdg:.5f}")

    return lat, lon, hdg
