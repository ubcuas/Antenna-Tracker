""" File that uses Socketio to listen to GCOM socket events"""

# note: Library installations were done quickly, maybe refactor to venv next time, espeically for reproducibility across machines
from typing import Any, List, Mapping
import socketio
sio = socketio.Client()

@sio.event
def connect():
    print("Connected")
    sio.emit("ping")

@sio.on("pong")
def pong():
    print("pong received from server")
    
@sio.event
def disconnect():
    print("Disconnected")
    exit(0)

# Connect to the telemetry server
sio.connect("http://localhost:8000",  transports=["websocket"])
sio.wait()
