import time
import serial

class AntennaTrackerSingleton():
    """Class representing the antenna tracker, there can only be one."""
    _antenna = None

    # SINGLETON CONSTRUCTION
    @classmethod
    def get_instance(cls):
        """Get instance or generate instance of singleton"""
        if not cls._antenna:
            cls._antenna = cls()
        return cls._antenna
    
    def __init__(self):
        """__new__ calls this function when a user calls the constructor of this class"""
        # initialize only once
        if not hasattr(self, "calibrated"):
            self.calibrated = False
            self.ser = serial.Serial(port="COM3", baudrate=9600)


    def flush_arduino(self):
        """
        gets any pending responses from the Arduino and prints them, the chain of
        responses will always end at a prompt for more input. So to skip to the next input 
        prompt all that ever needs to be done is to flush out all pending strings.
        """
        time.sleep(3)
        # temp for strings sent back from Arduino
        value_str = ""
        # while there is input to process from the Arduino, print it unti it ends
        # in waiting -> messages stored in a buffer that are yet to be printed
        while  self.ser.in_waiting > 0:
            value = self.ser.readline()
            value_str = str(value, "UTF-8")
            # if message is a string response from the Arduino
            print(value_str)

    def startup_calibrate(self):
        """
        Called by main, waits for the arduino to start up, flushes arduino, sends the y message to begin calibration.
        """
        self.flush_arduino()
        self.ser.write(bytes("y", "UTF-8"))
        # TODO: await response from arduino before doing this (ideally block the thread, but polling is fine ig)
        self.calibrated = True


    # send a drone update to the arduino
    def send_serial(self, posn_dict):
        # get user input if line returned by arduino requires some kind of response
        if(self.calibrated):
            message = f"{posn_dict['latitude']} , {posn_dict['longitude']}"
            self.ser.write(message.encode())