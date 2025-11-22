import requests
import time


url = "https://localhost:9000/api/drone/status"
def poll_gcom_status(url):
    while True:
        response = requests.get(url)
        if response.status_code == 200:
            status = response.json()
            print(status)
        else:
            print(f"Failed to get status, HTTP {response.status_code}")

        time.sleep(5) # Poll every 5 seconds
