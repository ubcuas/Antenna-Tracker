from typing import Any, List, Mapping
import socketio
import antenna_tracker_singleton, socket_events
import threading

# get the antenna tracker instance (ie: the only arduino connected to the computer via COM3 port)
antenna = antenna_tracker_singleton.AntennaTrackerSingleton()
# Connect to the telemetry server, run in async thread since this function will not terminate until the program does
threading.Thread(target=socket_events.gcom_connect, args=(antenna,), daemon=True).start()

# calibrate the tracker
antenna.startup_calibrate()

# poll infinitely so program doesn't end
while True:
   # TODO: Please refactor me this is god awfuk
   pass

