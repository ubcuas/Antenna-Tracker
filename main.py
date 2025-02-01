from typing import Any, List, Mapping
import socketio
import antenna_tracker_singleton, socket_events

# get the antenna tracker instance (ie: the only arduino connected to the computer via COM3 port)
antenna = antenna_tracker_singleton.AntennaTrackerSingleton()
antenna.startup_calibrate()

# Connect to the telemetry server
socket_events.gcom_connect(antenna)


