from dynamixel_sdk import *

class DynamixelInterface:
    def __init__(self, control_table, debug=True):
        self.control_table = control_table
        self.debug = debug
        self.motor_thresholds = {}
        self.motor_torque_states = {}
        self.operating_mode = {}

    def _print(self, msg):
        if self.debug:
            print(msg)

    def register_motor(self, DXL_ID, load_threshold=0):
        self.motor_torque_states[DXL_ID] = 1
        self.motor_thresholds[DXL_ID] = load_threshold

    def set_operating_mode(self, DXL_ID, output_type):
        # change operating mode for a dxl_id according to valid output types enum
        VALID_MOTOR_OUTPUT_TYPES = {
            'velocity':1,
            'position':3,
            'extended_position':4,
            'pwm':16
        }
        hex_key = VALID_MOTOR_OUTPUT_TYPES[output_type.lower()]
        if output_type in VALID_MOTOR_OUTPUT_TYPES.keys():
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.control_table['operating_mode'], VALID_MOTOR_OUTPUT_TYPES[output_type])
            self.operating_mode[DXL_ID] = output_type
            if dxl_comm_result != COMM_SUCCESS:
                self._print("SETTING OPERATING MODE: %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self._print("SETTING OPERATING MODE: %s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                self._print("SETTING OPERATING MODE FOR DXL_ID %s TO %s [OK]" % (DXL_ID, output_type.upper()))
        else:
            self._print("Invalid motor output type detected, please make sure to use one of: velocity, position, extended_position, pwm")

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
            self._print("TORQUE SET TO "+str(TORQUE_STATE)+" FOR DXL_"+str(DXL_ID))

    def set_goal_velocity(self, DXL_ID, GOAL_VELOCITY):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, self.control_table['goal_velocity'], GOAL_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            self._print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self._print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self._print("GOAL_VELOCITY SET TO "+str(GOAL_VELOCITY)+" FOR DXL_"+str(DXL_ID))
        pass

    def set_goal_position(self, DXL_ID, GOAL_POSITION):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, self.control_table['goal_position'], GOAL_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            self._print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self._print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            self._print("GOAL_VELOCITY SET TO "+str(GOAL_POSITION)+" FOR DXL_"+str(DXL_ID))
        pass

    # This function is not correct
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
        # self._print("[ID:%03d] Load: %d" % (DXL_ID, self.u16_to_i16(dxl_load)/10))
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