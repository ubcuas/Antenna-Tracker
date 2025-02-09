""" File that uses Socketio to listen to GCOM socket events"""

# note: Library installations were done quickly, maybe refactor to venv next time, espeically for reproducibility across machines
from typing import Any, List, Mapping
import socketio
import time
def gcom_connect(antenna):
    sio = socketio.Client()
    
    # Event listeners and handlers, all events run asynchronously
    @sio.event
    def connect():
        print("Connected")
        sio.emit("ping")
        while True:
            time.sleep(5)
            sio.emit("drone_update", {"timestamp" : 10, "latitude":20 , "altitude":10, "longitude" : 100, "vertical_velocity":10, "velocity":90, "heading":10, "battery_voltage":9 })

    @sio.on("pong")
    def pong():
        print("pong received from server")

    @sio.on("drone_update")
    def handletelemetry(dict):
        print("telemetry recieved")
        antenna.send_serial(dict)

    @sio.event
    def disconnect(arg):
        print("Disconnected")
        exit(0)

    sio.connect("http://localhost:8000",  transports=["websocket"])
    sio.wait()

