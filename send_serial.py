""" File that uses PySerial to send information to the Arduino"""
import time
import serial

## sketchy command line docs
## python -m serial.tools.list_ports (shows all serial ports your computer has access to)

# create instance of serial class from PySerial (current port is COM3 Arduino port)
ser = serial.Serial(port="COM3", baudrate=9600)


def flush_arduino():
    """ 
    EFFECT: gets any pending responses from the Arduino and prints them, the chain of
    responses will always end at a prompt for more input. So to skip to the next input 
    prompt all that ever needs to be done is to flush out all pending strings.
    """
    time.sleep(3)
    # temp for strings sent back from Arduino
    value_str = ""
    # while there is input to process from the Arduino, print it unti it ends
    while  ser.in_waiting > 0:
        value = ser.readline()
        value_str = str(value, "UTF-8")
        # if message is a string response from the Arduino
        print(value_str)


# keep reading serial inputs
while True:
    flush_arduino()
    # get user input if line returned by arduino requires some kind of response
    message = input("Response: ")
    ser.write(message.encode())