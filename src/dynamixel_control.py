import serial
import time
from DP2 import DP2

class dynamixel_control:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

    def connect(self):
        self.ser = serial.Serial(port=self.port,baudrate=self.baudrate)
        self.ser.isOpen()

    def is_connected(self):
        try:
            ser.inWaiting()
            return True
        except:
            print("Lost connection!")
            return False

    def enable_motors(self):
        self.write_serial(DP2(dynamixel_id=1, func_name='torque', value=True).packet)
        self.write_serial(DP2(dynamixel_id=2, func_name='torque', value=True).packet)

    def disable_motors(self):
        self.write_serial(DP2(dynamixel_id=1, func_name='torque', value=False).packet)
        self.write_serial(DP2(dynamixel_id=2, func_name='torque', value=False).packet)

    def motor_pos(self,motor_1_pos, motor_2_pos):
        self.write_serial(DP2(dynamixel_id=1, func_name='motor_pos', value=motor_1_pos).packet)
        self.write_serial(DP2(dynamixel_id=2, func_name='motor_pos', value=motor_2_pos).packet)

    # NOT SUPPORTED YET
    # TODO: Add motor speeds option to DP2.py
    def motor_speeds(self,motor_1_speed, motor_2_speed):
        return 0

    def write_serial(self, packet):
        self.ser.write(packet)
        # self.read_serial()

    def read_serial(self):
        print(self.ser.readline())

    def init_position(self):
        self.motor_pos(1024,1024)

    def test(self, LOWERLIMIT, UPPERLIMIT):
        for i in range(1024, LOWERLIMIT, -10):
            self.motor_pos(i,1024)
            time.sleep(0.1)
        for i in range(LOWERLIMIT, UPPERLIMIT, 10):
            self.motor_pos(i,1024)
            time.sleep(0.1)
        for i in range(UPPERLIMIT, 1024, -10):
            self.motor_pos(i,1024)
            time.sleep(0.1)
        for i in range(1024, LOWERLIMIT, -10):
            self.motor_pos(1024,i)
            time.sleep(0.1)
        for i in range(LOWERLIMIT, UPPERLIMIT, 10):
            self.motor_pos(1024,i)
            time.sleep(0.1)
        for i in range(UPPERLIMIT, 1024, -10):
            self.motor_pos(1024,i)
            time.sleep(0.1)

    def disconnect(self):
        self.disable_motors()
        self.ser.close()