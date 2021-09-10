#
#  Dynamixel Protocol 2.0 Packet Creator
#  Written for UBC UAS in 2021 by some guy
#  Interpreted from https://emanual.robotis.com/docs/en/dxl/protocol2/
#

from crccheck.crc import Crc16Umts

class DP2:
    def __init__(self, dynamixel_id, func_name, value=None):
        self.packet = bytearray()

        #Dynamixel 2.0 Protocol Header
        self.packet.extend([0xFF,0xFF,0xFD,0x00])

        #Dynamixel ID
        self.packet.append(int(hex(dynamixel_id), base=16))

        #packet instruction and parameters
        packet_content_options = {
            "ping":self.make_ping_packet,
            "torque":self.make_torque_packet,
            "motor_pos":self.make_motor_pos_packet
        }

        packet_content_func = packet_content_options.get(func_name, lambda: "Invalid func_name parameter for DP2 class.")
        if(value!=None):
            packet_content_func(value)
        else:
            packet_content_func()

        #CRC
        self.append_packet_crc()

    def make_ping_packet(self):
        self.packet.extend([0x06,0x00,0x03])

    def make_torque_packet(self, torque_enabled):
        # length, instruction, param
        self.packet.extend([0x06,0x00,0x03,0x40,0x00])
        # self.packet.append(0x00)

        if(torque_enabled):
            self.packet.append(0x01)
        else:
            self.packet.append(0x00)

    def make_motor_pos_packet(self, motor_pos):
        # length
        self.packet.append(0x09)
        self.packet.append(0x00)

        # instruction
        self.packet.append(0x03)

        # param p1
        self.packet.append(0x74)
        self.packet.append(0x00)

        # param p2
        motor_pos_hex = hex(motor_pos)[2:]
        if(len(motor_pos_hex)%2):motor_pos_hex="0"+motor_pos_hex
        paramlist = [int("0x"+motor_pos_hex[i:i+2], base=16) for i in range(0, len(motor_pos_hex), 2)][::-1]
        self.packet.extend(paramlist)

        # param p3
        self.packet.append(0x00)
        self.packet.append(0x00)

    def append_packet_crc(self):
        crcinst = Crc16Umts()
        crcinst.process(self.packet)
        crchex = crcinst.finalhex()
        crclist = [int("0x"+crchex[i:i+2], base=16) for i in range(0, len(crchex), 2)][::-1]
        self.packet.extend(crclist)
