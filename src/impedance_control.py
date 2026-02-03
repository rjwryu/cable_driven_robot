import can
import struct
import time
import sys
import os

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = 'can0'
INTERFACE = 'socketcan'


#Torque Constant and Friction Gap
TORQUE_CONSTANT = 0.59 #Nm/A
FRICTION_CURRENT = 0.25 #0.45/2


SENSOR_CALIBRATION_RATIO = 1

# --- HARD SAFETY LIMITS ---
MAX_POS_ERROR = 90.0
SAFE_CURRENT_LIMIT = 3.8
MAX_VELOCITY_LIMIT = 3000.0 

# --- TUNING ---
Kp = 0.4        # A/Deg
Kd = 0.008       # A/(DEg/s)
Kf = 0          
FRICTION_RAMP_WIDTH = 100 

class MotorUnwrapper:
    def __init__(self):
        self.last_raw = None
        self.turns = 0

    def update(self, raw_pos):
        if self.last_raw is None:
            self.last_raw = raw_pos
        diff = raw_pos - self.last_raw
        if diff < -8192: self.turns += 1
        elif diff > 8192: self.turns -= 1
        self.last_raw = raw_pos
        
        base_angle = (self.turns * 360.0) + ((raw_pos / 16384.0) * 360.0)
        return base_angle * (360 / 8)

def reset_can_interface():
    os.system(f"sudo ip link set {CHANNEL} down 2>/dev/null")
    os.system(f"sudo ip link set {CHANNEL} up type can bitrate 1000000 2>/dev/null")

def send_torque(bus, current):
    current = max(-SAFE_CURRENT_LIMIT, min(SAFE_CURRENT_LIMIT, current))
    current_int = int(current / 0.01)
    data = struct.pack('<Bxxxhxx', 0xA1, current_int) 
    try:
        bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
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
        raw_speed_int = struct.unpack('<h', latest_msg.data[4:6])[0]
        raw_pos_int = struct.unpack('<H', latest_msg.data[6:8])[0] 
        
        meas_current = (raw_current_int * 0.01) * SENSOR_CALIBRATION_RATIO
        meas_speed = raw_speed_int * 1.0  # deg/s
        
        return raw_pos_int, meas_speed, meas_current

    return None, None, None

def run_safe_impedance():
    print(f"--- IMPEDANCE CONTROL (NO VELOCITY FILTER) ---")
    print(f"Sensor Correction Factor: {SENSOR_CALIBRATION_RATIO}")
    reset_can_interface()
    
    try:
        bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=1000000)
    except:
        return

    unwrapper = MotorUnwrapper()
    print("Searching for motor...")

    raw_p = None
    while raw_p is None:
        send_torque(bus, 0)
        time.sleep(0.01)
        raw_p, _, _ = get_latest_state(bus)
        if raw_p is None:
            time.sleep(0.5)
        else:
            print("\n✅ CONNECTED!")

    # -------- ZERO REFERENCE --------
    absolute_start_pos = unwrapper.update(raw_p)
    zero_offset_pos = absolute_start_pos
    print(f"Zeroed at current pose → 0.0°")

    target_pos = absolute_start_pos
    loop_count = 0
    target_torque = 0.0

    deg2rad = 3.141592653589793 / 180.0

    try:
        while True:
            send_torque(bus, target_torque)
            
            raw_p, raw_v, meas_current = get_latest_state(bus)
            
            if raw_p is not None:
                absolute_pos = unwrapper.update(raw_p)

                

                # ---- RELATIVE FRAME ----
                current_pos = absolute_pos - zero_offset_pos
                target_rel = target_pos - zero_offset_pos   # = 0
                error =  current_pos - target_rel
                
                # --- SAFETY ---
                if abs(error) > MAX_POS_ERROR:
                    print(f"\nSAFETY STOP: Twisted too far ({error:.1f}°)")
                    send_torque(bus, 0)
                    break
                
                if abs(raw_v) > MAX_VELOCITY_LIMIT:
                    print(f"\nSAFETY STOP: Overspeed ({raw_v:.1f} deg/s)")
                    send_torque(bus, 0)
                    break

                # --- CONTROL LAW (RAW VELOCITY) ---
                err_clamped = max(-FRICTION_RAMP_WIDTH, min(FRICTION_RAMP_WIDTH, error))
                f_term = Kf * (err_clamped / FRICTION_RAMP_WIDTH)

                target_torque = - (Kp * error) - (Kd * raw_v) - f_term #in AMP change raw_v to v_error and error to p_error, calc rough estimate of units for kp kd
                
                # --- VISUALIZER ---
                loop_count += 1
                if loop_count >= 50:
                    sys.stdout.write(
                        f"\r"
                        f"Pos Act: {current_pos:6.1f}° ({current_pos*deg2rad:6.3f} rad) | "
                        f"Pos Tgt: {0.0:6.1f}° ({0.0:6.3f} rad) | "
                        f"Vel Act: {raw_v:6.1f}°/s ({raw_v*deg2rad:6.3f} rad/s) | "
                        f"Cur Cmd: {target_torque:5.2f} A | "
                        f"Cur Act: {meas_current:5.2f} A"
                    )
                    sys.stdout.flush()
                    loop_count = 0
            
            time.sleep(0.002) #500Hz

    except KeyboardInterrupt:
        send_torque(bus, 0)
        print("\nStopped.")

if __name__ == "__main__":
    run_safe_impedance()
