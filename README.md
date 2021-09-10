# Sunflower

This repository holds the code required to run the UBC UAS Antenna Tracker.

### Dependencies
**Dynamixel SDK**: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
**Adafruit CircuitPython LSMDS91**: https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1

### Design History
#### Prior to September 2020
Sunflower is known as Antenna Tracker.
https://gitlab.com/ubcuas/rcomms/rcomms2019

| Key  | Value |
| ------------- |:-------------:|
| Computer      | Raspberry Pi 3     |
| Microcontroller      | OpenCM 9.04     |

#### September 2020 - Present
Sunflower experienced major hardware and software changes.

| Key  | Value |
| ------------- |:-------------:|
| Computer      | Odroid XU4     |
| USB Converter      | U2D2     |

The current conne
### Odroid XU4 Common Issues and Resolutions
This code uses Adafruit CircuitPython

### Constants
#### USB Device Names
These port names can be found by checking which USB interface Linux has assigned to the devices.
`U2D2_PORT_NAME = "/dev/ttyUSB0"`
`GPS_PORT_NAME = "/dev/ttyUSB1"`

#### Dynamixel Control Table
This control table is dependent on the motor model.
For more information, consult this website: https://emanual.robotis.com/docs/en/dxl/
Your control table will be under `2. Control Table` on your motor's webpage.

```
xl430_w250_control_table = {
    'torque_enable':64,
    'goal_velocity':104,
    'goal_position':116,
    'load':126,
    'velocity':128,
    'position':132,
    'max_position':48,
    'min_position':52,
    'velocity_limit':44
}
```
The table above was extracted from this page:
https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-data-address

### Dynamixel Control
Code used to control the motors can be found in **dynamixel_interface.py**
#### For a group of dynamixels, you will need to:
Initialize an interface like so:
```
# Initialize the interface on U2D2_PORT_NAME with baudrate of 57600 using
# Dynamixel Protocol 2.0
motors = DynamixelInterface(xl430_w250_control_table, debug=True)
motors.connect(U2D2_PORT_NAME, 57600, 2.0)
```
**It is currently assumed that both motors are the same model.**
The interface requires a control table for initialization.
The debug parameter can be set to `True` for verbose output.
It is `False` by default.


#### For any dynamixel in this group, you will need to:
- Register it using their ID
- Set your desired load limit `motors.register_motor(1, load_threshold=5)`
-

### Issues
If you come across a bug or believe that something is lacking, feel free to create an issue.
Direct any questions to the maintainer specified below:
> Contact **Alexander Chudinov** at `alexander.chudinov@icloud.com` or on Slack
