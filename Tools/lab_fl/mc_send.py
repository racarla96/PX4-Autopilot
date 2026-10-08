#!/usr/bin/env python3
"""Virtual joystick for SITL: centered sticks over MAVLink MANUAL_CONTROL.

LAB and FL require a manual control link (the safety pilot must be able to take over), and the simulator
has no radio. Not needed when QGroundControl's virtual joystick or a real joystick is enabled.
"""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection('udpout:127.0.0.1:18570', source_system=255, source_component=190)
n = 0
while True:
    if n % 20 == 0:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    m.mav.manual_control_send(1, 0, 0, 500, 0, 0)  # x, y, z (throttle 50 %), r, buttons
    n += 1
    time.sleep(0.05)
