import time
from pymavlink import mavutil

# --- CONFIGURATION ---
# /dev/ttyACM0 is the standard for Pixhawk connected via USB to Jetson
# Baud rate 115200 is default for USB MAVLink
connection_string = '/dev/ttyACM0'
baud_rate = 115200

print(f"Connecting to Pixhawk on: {connection_string} at {baud_rate} baud...")

# Create the connection
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Wait for a heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from System {master.target_system} Component {master.target_component}")

def set_rc_channel_pwm(channel, pwm):
    """ Sets RC channel PWM value. 1500=Neutral, 1100=Low, 1900=High """
    rc_channel_values = [65535] * 18
    rc_channel_values[channel - 1] = pwm
    
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values
    )

def arm_vehicle():
    """ Sends an arming command to the Pixhawk """
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    # Wait for acknowledgement
    ack = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"Arming result: {ack.result}")

def disarm_vehicle():
    """ Sends a disarming command to the Pixhawk """
    print("Disarming vehicle...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

def main():
    try:
        print("\n!!! SAFETY WARNING: Prop the car wheels off the ground !!!")
        print("Starting test in 3 seconds...")
        time.sleep(3)

        # ArduRover usually only responds to throttle if in a manual-capable mode
        print("Setting Mode to MANUAL...")
        master.set_mode('MANUAL')

        # MANDATORY FOR THROTTLE: Arm the vehicle
        arm_vehicle()

        # Test Steering (Channel 1) and Throttle (Channel 3)
        for chan in [1, 3]:
            print(f"\n--- Testing Channel {chan} ---")
            
            # Move to High
            print(f"Channel {chan} -> 1900 (High/Right/Forward)")
            set_rc_channel_pwm(chan, 1900)
            time.sleep(1.5)
            
            # Move to Low
            print(f"Channel {chan} -> 1100 (Low/Left/Reverse)")
            set_rc_channel_pwm(chan, 1100)
            time.sleep(1.5)
            
            # Return to Neutral
            print(f"Channel {chan} -> 1500 (Neutral)")
            set_rc_channel_pwm(chan, 1500)
            time.sleep(1)

        print("\nTest complete. All channels reset.")

    except KeyboardInterrupt:
        print("\nUser stopped the test.")
    finally:
        # Safety: Disarm at the end of the test
        disarm_vehicle()
        # Emergency reset: Set all channels to 1500 and clear override
        master.mav.rc_channels_override_send(master.target_system, master.target_component, *[1500]*18)
        print("Overrides cleared and vehicle disarmed.")

if __name__ == "__main__":
    main()