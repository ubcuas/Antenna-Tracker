import serial
import pynmea2

class GPSInterface:
    def __init__(self, port, baudrate):
        self.port = port
        self.gps_data = {
            "lat":None,
            "lng":None,
            "alt_msl":None,
            "num_sats":None,
        }

        try:
            self.ser = serial.Serial(port, baudrate)
        except:
            self.ser = None
            print("Could not connect to GPS on port [error]: %s", self.port)

    def nmea_handler(self, nmea_obj):

        def handle_vtg(msg):
            return "Handled VTG"

        def handle_gga(msg):
            if msg.lat_dir == 'N':
                self.gps_data["lat"] = round(float(msg.lat)/100,8)
            else:
                self.gps_data["lat"] = round(float(msg.lat)/100,8)*-1

            if msg.lon_dir == 'E':
                self.gps_data["lng"] = round(float(msg.lon)/100,8)
            else:
                self.gps_data["lng"] = round(float(msg.lon)/100,8)*-1

            self.gps_data["num_sats"] = int(msg.num_sats)
            self.gps_data["alt_msl"] = float(msg.altitude)

            return "Handled GGA"

        def handle_gsa(msg):
            return "Handled GSA"

        def handle_gsv(msg):
            return "Handled GSV"

        def handle_gll(msg):
            return "Handled GLL"

        def handle_rmc(msg):
            return "Handled RMC"

        def handle_txt(msg):
            return "Handled TXT"

        nmea_switcher = {
            pynmea2.types.talker.VTG: handle_vtg,
            pynmea2.types.talker.GGA: handle_gga,
            pynmea2.types.talker.GSA: handle_gsa,
            pynmea2.types.talker.GSV: handle_gsv,
            pynmea2.types.talker.GLL: handle_gll,
            pynmea2.types.talker.RMC: handle_rmc,
            pynmea2.types.talker.TXT: handle_txt
        }

        nmea_fn = nmea_switcher.get(type(nmea_obj), lambda x: "Handling for "+repr(type(x))+" is not implemented")
        return nmea_fn(nmea_obj)

    def has_data(self):
        if None not in self.gps_data.values():
            return True
        return False

    def read(self):
        if self.ser is None:
            return 0
        nmea = self.ser.readline().decode('ascii', errors='replace').strip()
        try:
            nmeaobj = pynmea2.parse(nmea)
            try:
                self.nmea_handler(nmeaobj)
            except:
                print("[GPS] NMEA Handler Error [warning]: ", repr(type(nmeaobj)))
        except:
            print("[GPS] Invalid Data Received [warning]: ", nmea)