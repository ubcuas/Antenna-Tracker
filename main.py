from typing import Any, List, Mapping
import socketio
import antenna_tracker_singleton, socket_events
import threading
import gcom_status_poller

# get the antenna tracker instance (ie: the only arduino connected to the computer via COM3 port)
antenna = antenna_tracker_singleton.AntennaTrackerSingleton()

# calibrate the tracker
antenna.startup_calibrate()
antenna.poll_gcom()

