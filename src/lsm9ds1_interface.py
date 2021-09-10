class LSM9DS1Interface:
    def __init__(self, i2c_addr, i2c_port):
        print("Initialized I2C"+str(i2c_port)+" with addr 0x"+str(i2c_addr))
        self.sensor_data = {
            "accelerometer":{
                "x":None,
                "y":None,
                "z":None
            },
            "magnetometer":{
                "x":None,
                "y":None,
                "z":None
            },
            "gyroscope":{
                "x":None,
                "y":None,
                "z":None
            }
        }

    def read(self):
        print("read data")