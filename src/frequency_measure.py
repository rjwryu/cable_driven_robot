import can
import struct
import time
import os

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = "can0"
INTERFACE = "socketcan"

SENSOR_CALIBRATION_RATIO = 1.0
SAFE_CURRENT_LIMIT = 0.0   # force zero torque

def reset_can_interface():
    os.system(f"sudo ip link set {CHANNEL} down 2>/dev/null")
    os.system(f"sudo ip link set {CHANNEL} up type can bitrate 1000000 2>/dev/null")

def send_zero_current(bus):
    current_int = 0
    data = struct.pack('<Bxxxhxx', 0xA1, current_int)
    try:
        bus.send(can.Message(
            arbitration_id=MOTOR_ID,
            data=data,
            is_extended_id=False
        ))
    except:
        pass

def get_latest_state(bus):
    latest_msg = None
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            break
        if msg.arbitration_id == REPLY_ID:
            latest_msg = msg

    if latest_msg:
        raw_current_int = struct.unpack('<h', latest_msg.data[2:4])[0]
        meas_current = raw_current_int * 0.01 * SENSOR_CALIBRATION_RATIO
        return meas_current

    return None

def main():
    print("--- MOTOR CURRENT READOUT (ZERO TORQUE KEEPALIVE) ---")
    reset_can_interface()

    try:
        bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=1000000)
    except Exception as e:
        print("CAN open failed:", e)
        return

    prev_time = None

    while True:
        # --- trigger motor feedback ---
        send_zero_current(bus)

        current = get_latest_state(bus)

        if current is not None:
            now = time.time()
            if prev_time is not None:
                dt = now - prev_time
                freq = 1.0 / dt
                print(
                    f"Current: {current:6.3f} A | "
                    f"Update rate: {freq:6.1f} Hz",
                    end="\r"
                )
            prev_time = now

        time.sleep(0.001)  # same pacing as your control loop

if __name__ == "__main__":
    main()
