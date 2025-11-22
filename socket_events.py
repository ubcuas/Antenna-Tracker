""" File that uses Socketio to listen to GCOM socket events"""

# note: Library installations were done quickly, maybe refactor to venv next time, espeically for reproducibility across machines
from typing import Any, List, Mapping
import socketio
import time


def gcom_connect(antenna):
    # initialize locals
    sio = socketio.Client()
    init_pos_sent = False

    # Event listeners and handlers, all events run asynchronously
    @sio.event
    def connect():
        print("Connected")
        sio.emit("ping")
        count = 0
        while True:
            count += 10
            time.sleep(.1)
            sio.emit("drone_update", {"timestamp" : 10, "latitude":40 + count, "altitude":10, "longitude" : 40 + count, "vertical_velocity":10, "velocity":90, "heading":10, "battery_voltage":9 })

    @sio.on("pong")
    def pong():
        print("pong received from server")

    @sio.on("drone_update")
    def handletelemetry(dict):
        # nonlocal defn ensures variable references local in encapsulating function
        nonlocal init_pos_sent
        print("telemetry recieved")
        # send initial position for CALIBRATION procedure
        if not init_pos_sent:
            # TODO: Remove these fixed coords when testing with GCOM & Drone
            dict['longitude'] = -123.1264
            dict['latitude'] = 49.3410
            dict['altitude'] = 10
            antenna.send_serial(dict, True)
            init_pos_sent = True
            return
        antenna.send_serial(dict)

    @sio.event
    def disconnect(arg):
        print("Disconnected")
        exit(0)

    sio.connect("http://localhost:8000",  transports=["websocket"])
    sio.wait()

