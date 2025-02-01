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
            # cls = class instance, cls() = class constructor call
            cls._antenna = cls()
        return cls._antenna
    def __init__(self):
        """__new__ decides if this function is called when a user instantiates the class"""
        # initialize only once
        if not hasattr(self, "calibrated"):
            self.calibrated = False
            self.ser = serial.Serial(port="COM3", baudrate=9600)
            self.positions = []
            
    def flush_arduino(self):
        """
        gets any pending responses from the Arduino and prints them, the chain of
        responses will always end at a prompt for more input. So to skip to the next input 
        prompt all that ever needs to be done is to flush out all pending strings.
        """
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
        # static boot timer to allow arduino to execute 
        time.sleep(10)
        self.flush_arduino()
        self.ser.write(bytes("y", "UTF-8"))
        # TODO: await response from arduino before doing this (ideally block the thread, but polling is fine ig)
        while self.ser.in_waiting <= 0:
            pass
        value = self.ser.readline()
        print(str(value, "UTF-8"))
        self.calibrated = True
       

    # TODO: As asynchronous drone_update events are passed in to the control of the program, we want to execute only the most recent one
    # when the tracker is ready.
    # 1. We will measure readiness by reading strings from the serial port and looking for the substring "AWAITING CONTROL INPUT".
    # 2. As threads arrive they will set a POSITION field in the antenna tracker.
    # 3. When the antenna tracker is ready to move again it will read this field and move there
    # This should deal with asynchrony issues and overlapping drone_update_events.
    def send_serial(self, posn_dict):
        """
        send a drone update to the arduino, multiple threads could be running this asynchronously
        we want the last thread initialized to have its position sent, then dump the rest.
        """
        # get user input if line returned by arduino requires some kind of response
        if(self.calibrated and self.ser.in_waiting > 0):
            value = self.ser.readline()
            print(str(value, "UTF-8"))
            message = f"{posn_dict['latitude']} , {posn_dict['longitude']}\n"
            self.ser.write(message.encode())