import time
from dynamixel_interface import DynamixelInterface
from gps_interface import GPSInterface
from lsm9ds1_interface import LSM9DS1Interface

U2D2_PORT_NAME = "/dev/tty.usbserial-FT4TFLAV"
GPS_PORT_NAME = "/dev/tty.usbmodem14401"
LSM9DS1_I2C_ADDR = 0xe6
LSM9DS1_I2C_BUS = 5


xl430_w250_control_table = {
    'torque_enable':64,
    'goal_velocity':104,
    'goal_position':116,
    'load':126,
    'velocity':128,
    'position':132,
    'max_position':48,
    'min_position':52,
    'velocity_limit':44
}

motors = DynamixelInterface(xl430_w250_control_table, debug=True)
motors.connect(U2D2_PORT_NAME, 57600, 2.0)
motors.register_motor(1, load_threshold=5)
motors.register_motor(2, load_threshold=10)

# gps = GPSInterface(GPS_PORT_NAME, 9600)

multisensor = LSM9DS1Interface(LSM9DS1_I2C_ADDR, LSM9DS1_I2C_BUS)

# print("GPS [...]")

# while not gps.has_data():
#     gps.read()

# print("GPS [OK]")

while(1):
    time.sleep(0.2)
    motors.run()
    # gps.read()
    # multisensor.read()
    # print(repr(gps.gps_data))