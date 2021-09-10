# Read GPS
# Communicate with Dynamixels
# Limit Movement
import atexit
import sys
import os
import time
# Add switch for emergency
# Secure components
from dynamixel_control import dynamixel_control

u2d2 = dynamixel_control("/dev/tty.usbserial-FT4TFLAV", 1000000)

def main():
    u2d2.connect()
    u2d2.enable_motors()
    u2d2.init_position()
    # time.sleep(2)
    # u2d2.test(592,1456)

def shutdown():
    print('\nShutting Down...')
    u2d2.disable_motors()
    print("\tMotors Disabled [OK]")
    u2d2.disconnect()
    print("\tU2D2 Disconnected [OK]")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        shutdown()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)


# messagedynamixel
#  -> wait for reply or smtn

# readgps
# read sensor
#  -> wait for reply or smtn

# calculatetargetvector
# coord system translation