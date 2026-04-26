from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("Connected!")

master.arducopter_arm()
time.sleep(2)

IGNORE = 65535

def move(steering, throttle):
    """
    steering: 1000=full left, 1500=straight, 2000=full right
    throttle: 1000=full reverse, 1500=stop, 2000=full forward
    """
    vals = [IGNORE] * 18
    vals[0] = steering   # Channel 1 = Servo (steering)
    vals[2] = 3000 - throttle   # Channel 3 = DC Motor (throttle)
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *vals
    )

def stop():
    move(1500, 1500)

# --- TEST MOVEMENTS ---

print("Forward...")
for _ in range(20):
    move(1500, 1700)
    time.sleep(0.1)
stop()
time.sleep(1)

print("Reverse...")
for _ in range(20):
    move(1500, 1300)
    time.sleep(0.1)
stop()
time.sleep(1)

print("Turn Right...")
for _ in range(20):
    move(1900, 1500)
    time.sleep(0.1)
stop()
time.sleep(1)

print("Turn Left...")
for _ in range(20):
    move(1100, 1500)
    time.sleep(0.1)
stop()

print("Done.")