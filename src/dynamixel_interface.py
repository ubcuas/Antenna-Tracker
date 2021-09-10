from dynamixel_sdk import *

class DynamixelInterface:
    def __init__(self, control_table, debug=True):
        self.control_table = control_table
        self.debug = debug
        self.motor_thresholds = {}
        self.motor_torque_states = {}

    def _print(self, msg):
        if self.debug:
            print(msg)

    def register_motor(self, DXL_ID, load_threshold=0):
        self.motor_torque_states[DXL_ID] = 1
        self.motor_thresholds[DXL_ID] = load_threshold

    def connect(self, DEVICENAME, BAUDRATE, PROTOCOL_VERSION):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            self._print("Succeeded to open the port")
        else:
            self._print("Failed to open the port")

        if self.portHandler.setBaudRate(BAUDRATE):
            self._print("Succeeded to change the baudrate")
        else:
            self._print("Failed to change the baudrate")

    def set_torque_state(self, DXL_ID, TORQUE_STATE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.control_table['torque_enable'], TORQUE_STATE)
        if dxl_comm_result != COMM_SUCCESS:
            self._print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self._print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self._print("Torque set to "+str(TORQUE_STATE)+" for DXL_"+str(DXL_ID))

    def u16_to_i16(self, x):
        # Unsigned Decimal in Two's Complement -> Signed Decimal
        if(x>32767):
            return x - 65535
        return x

    def read_motor_load(self, DXL_ID):
        # Read motor load as a %
        # This number will not be accurate if the torque is off
        dxl_load, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['load'])
        if dxl_comm_result != COMM_SUCCESS:
            self._print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self._print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        self._print("[ID:%03d] Load: %d" % (DXL_ID, self.u16_to_i16(dxl_load)/10))
        return self.u16_to_i16(dxl_load)/10

    def enforce_motor_load_threshold(self):
        # Check that motors do not exert too much force.
        # Disable them if they do.
        for entry in self.motor_thresholds.items():
            DXL_ID, threshold = entry
            if(abs(self.read_motor_load(DXL_ID))>threshold):
                if(self.motor_torque_states[DXL_ID]!=0):
                    self.set_torque_state(DXL_ID, 0)
                    self.motor_torque_states[DXL_ID]=0
            else:
                if(self.motor_torque_states[DXL_ID]!=1):
                    self.set_torque_state(DXL_ID, 1)
                    self.motor_torque_states[DXL_ID]=1

    def run(self):
        self.enforce_motor_load_threshold()