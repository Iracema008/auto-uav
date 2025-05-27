import time
import busio
import board
import adafruit_rfm9x

from digitalio import DigitalInOut
from pymavlink import mavutil

# Replace with your serial port and baudrate (Pixhawk default is 57600)
serial_port = '/dev/ttyAMA0'
baudrate =  57600

# Configure RFM9x LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

#print("Found rfm9x module version:", adafruit_rfm9x.version)

rfm9x=None
# sets up rfm9x module
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
    print('RFM9x: Detected')
    rfm9x.tx_power = 23
    print('RFM9x: Set Tx Power to 23dB')
except RuntimeError as error:
    print('RFM9x Error: ', error)
    exit()
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    exit()

def send_data(command):
    packet = bytes(command, "utf-8")
    rfm9x.send(packet)

    print(f"Sent command: {command}")
    time.sleep(0.1)

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(serial_port, baud=baudrate)

print("Waiting for heartbeat...")
master.wait_heartbeat()

#using recv_msg right now to listen to ANY mavlink msg
print(f"Heartbeat received from system (ID {master.target_system}, component {master.target_component})")


# 3) Loop, grabbing GPS_RAW_INT messages
print("Waiting for GPS_RAW_INT packets...\n(Press Ctrl-C to exit)\n")
while True:
    # blocking=True will wait until a GPS_RAW_INT is received
    print("Entered while")

    #msg = master.recv_msg()
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)

    print(msg)

    if not msg:
        print ("not msg")
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

    # coordanates as utf-string to send to lora
    send_data(f"{lat:.7f},{lon:.7f},{alt:.2f}")

