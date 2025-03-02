import time
import serial
import threading

class AntennaTrackerSingleton():
    """Class representing the antenna tracker, there can only be one."""
    _antenna = None

    # SINGLETON CONSTRUCTION
    # override object class' memory alloc function (bottom of call stack when you call class constructor)
    def __new__(cls):
        """Get instance or generate instance of singleton"""
        # cls = class instance, cls() = class constructor call (new)
        if not cls._antenna:
            # call Python's default object class allocation function and pass in this class (By default when you call
            # any constructor the OBJECT class instance of __new__ [with the ability to allocate mem] is called)
            cls._antenna = super().__new__(cls)
        return cls._antenna

    def __init__(self):
        """__new__ decides if this function is called when a user instantiates the class"""
        # initialize only once
        # public fields
        self.calibrated = False
        self.ser = serial.Serial(port="COM3", baudrate=9600) # starts the arduino sketch
        self.awaitingInput = False
        self.initial_telemetry = {} # empty dict

        # private fields
        self._awatingInputLock = threading.Lock()

        # create thread for flush_arduino, daemon = true => shutdown thread on main thread exit
        message_thread = threading.Thread(target=self._flush_arduino, daemon=True)
        message_thread.start()
    
# PRIVATE METHODS
    # perpetually runs asynchronusly in the background
    def _flush_arduino(self):
        """
        gets any pending responses from the Arduino and prints them, the chain of
        responses will always end at a prompt for more input. So to skip to the next input 
        prompt all that ever needs to be done is to flush out all pending strings.
        """
        # while there is input to process from the Arduino, print it unti it ends
        # in waiting -> messages stored in a buffer that are yet to be printed
        while True:
            # auto locks and releases
            value = self.ser.readline()
            value_str = str(value, "UTF-8")
            # if message is a string response from the Arduino
            print(value_str)
            if "AWAITING INPUT" in value_str:
                print("tracker waiting for input, next thread should send")
                self.awaitingInput = True
                
# PUBLIC METHODS
    def startup_calibrate(self):
        """
        Called by main, waits for the arduino to start up, flushes arduino, sends the y message to begin calibration.
        """
        # static boot timer to allow arduino to execute 
        time.sleep(5)
        self.ser.write(bytes(f"{self.initial_telemetry['latitude']} , {self.initial_telemetry['longitude']}\n", "UTF-8"))
        time.sleep(5)
        self.ser.write(bytes("y", "UTF-8"))
        # TODO: await response from arduino before doing this (ideally block the thread, but polling is fine ig; USE MESSAGING THREAD TO DO THIS???)
        while self.ser.in_waiting <= 0:
            pass
        self.calibrated = True
       

    # TODO: As asynchronous drone_update events are passed in to the control of the program, we want to execute only the most recent one
    # when the tracker is ready.
    # 1. We will measure readiness by reading strings from the serial port and looking for the substring "AWAITING INPUT" (done in a perpetual async loop).
    # 2. As threads arrive they will aquire a lock and check if drone is calibrated and ready for input, the thread will end with no action if the lock is aquired and
    #       the conditions are not met.
    # 3. When the antenna tracker is ready to move again, the next thread that aquires the lock will send a position to the tracker
    # This should deal with asynchrony issues and overlapping drone_update_events.

    # mutex send_serial and perpetual async loop.
    def send_serial(self, posn_dict, initial=False):
        """
        send a drone update to the arduino, multiple threads could be running this asynchronously
        we want the last thread initialized to have its position sent, then dump the rest.
        """
        # simply save position for calibration if its the first transmission
        # DO NOT MOVE DRONE UNTIL AFTER CALIBRATION, AND ONLY WHILE GCOM IS RECIEVING POSN
        if initial:
            self.initial_telemetry = posn_dict
            return
        # auto acquires and releases lock
        with self._awatingInputLock:
            print("serial LOCKED IN")
            # get user input if line returned by arduino requires some kind of response
            if(self.calibrated and self.awaitingInput):
                message = f"{posn_dict['latitude']} , {posn_dict['longitude']}\n"
                self.ser.write(message.encode())
                self.awaitingInput = False
                print(f"Sent: {message}")
      