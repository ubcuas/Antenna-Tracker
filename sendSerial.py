import serial

## sketchy command line docs
## python -m serial.tools.list_ports (shows all serial ports your computer has access to)

# create instance of serial class from PySerial (current port is COM3 Arduino port)
#       WTF is a baudrate
ser = serial.Serial(port="COM3", baudrate=9600)


# EFFECT: will skip any lines we dont want a user/GCOM to respond to
def getNextInputLocation():
    # temp for strings sent back from Arduino
    valueStr = ""

    while (not valueStr.find("(USER/GCOM RESPONSE)")):
        # if message is a string response from the Arduino
        if("()")




# keep reading serial inputs
while True:
    # read feedback sent from serial port (arduino)
    value = ser.readline()
    # convert serial message to UTF-8 string
    valueStr = str(value, "UTF-8")
    # print message sent by arduino
    print(valueStr)

    # get user input if line returned by arduino requires some kind of response
    message = input("Response: ")
    ser.write(message.encode())


