import time
import busio
import board
import adafruit_rfm9x

from digitalio import DigitalInOut
from pymavlink import mavutil

import RPi.GPIO as GPIO

# Configure RFM9x LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

lat = 1
lon = 2


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


def send_rfm(lat, lon):
    rfm9x.send(bytes(f"{lat},{lon}", 'utf-8'))
