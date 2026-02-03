#Friction=Base Stiction+(Scaling Factor×Gravity Force)

import can
import struct
import time
import sys
import os
import math

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = 'can0'
INTERFACE = 'socketcan'

# --- FINAL CALIBRATED PHYSICS CONSTANTS (Jan 31 - Gold Standard) ---
# Source: "Flying Off/Falling Down" Limit Test
TORQUE_CONSTANT = 0.59    # Nm/A (Slope from 0.2kg to 0.83kg data)

# --- FRICTION MODEL CONSTANTS (Derived from Data) ---
# Friction = Base + (Slope * Load)
FRICTION_INTERCEPT = 0.1 # Base Stiction (Amps)
FRICTION_SLOPE = 0.05     # Scaling Factor (Amps friction per Amp load)

# --- ROBOT PHYSICAL PROPERTIES ---
# Update these two numbers whenever you change the weight!
ARM_MASS_KG = 0.41        # Total Mass (Arm 0.2 + Weight 0.43)
COM_RADIUS_M = 0.17162      # Distance to Center of Mass (meters)

SENSOR_CALIBRATION_RATIO = 1

# --- HARD SAFETY LIMITS ---
MAX_POS_ERROR = 180.0
SAFE_CURRENT_LIMIT = 5.0  
MAX_VELOCITY_LIMIT = 3000.0 

# --- TUNING ---
Kp = 0.4        # A/Deg (Stiffness)
Kd = 0.008        # A/(Deg/s) (Damping)
Kf = 0.0        # Keep 0 (We use explicit friction_ff_amps instead)

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
    print(f"--- IMPEDANCE CONTROL WITH DYNAMIC FRICTION SCALING ---")
    print(f"Calibrated Kt: {TORQUE_CONSTANT} Nm/A")
    print(f"Friction Model: {FRICTION_INTERCEPT}A + ({FRICTION_SLOPE} * Load)")
    print(f"Gravity Comp Mass: {ARM_MASS_KG} kg @ {COM_RADIUS_M} m")
    
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
    print(f"READY. Press CTRL+C to stop.")

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
                target_rel = target_pos - zero_offset_pos
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

                # --- 1. GRAVITY FEEDFORWARD ---
                # Physics: Torque = m * g * r * sin(theta)
                current_rad = current_pos * deg2rad
                grav_torque_nm = ARM_MASS_KG * 9.81 * COM_RADIUS_M * math.sin(current_rad)
                
                # Convert to Current
                gravity_comp_amps = grav_torque_nm / TORQUE_CONSTANT

                # --- 2. DYNAMIC FRICTION COMPENSATION (LOAD SCALED) ---
                # Step A: Calculate how much load is on the gears right now
                # We use the absolute value of gravity compensation as the "Load Proxy"
                current_load_amps = abs(gravity_comp_amps)
                
                # Step B: Apply Linear Scaling Model (Derived from your Data)
                # Friction increases as gears press harder together
                dynamic_friction_mag = FRICTION_INTERCEPT + (FRICTION_SLOPE * current_load_amps)
                
                # Step C: Apply Direction (Oppose Error / Support PID)
                if abs(error) > 0.5:
                    if error < 0:
                        friction_ff_amps = dynamic_friction_mag
                    else:
                        friction_ff_amps = -dynamic_friction_mag
                else:
                    friction_ff_amps = 0.0

                # --- 3. CONTROL LAW ---
                # Total = PID + Gravity + Dynamic Friction
                pid_term = - (Kp * error) - (Kd * raw_v)
                
                target_torque = pid_term + gravity_comp_amps + friction_ff_amps
                
                # --- VISUALIZER ---
                loop_count += 1
                if loop_count >= 50:
                    sys.stdout.write(
                        f"\r"
                        f"Err: {error:5.1f}° | "
                        f"Grav: {gravity_comp_amps:4.2f} A | "
                        f"DynFric: {friction_ff_amps:4.2f} A | "
                        f"Total: {target_torque:5.2f} A |"
                        f"Cur Act: {meas_current:5.2f} A"
                    )
                    sys.stdout.flush()
                    loop_count = 0
            
            time.sleep(0.002)

    except KeyboardInterrupt:
        send_torque(bus, 0)
        print("\nStopped.")

if __name__ == "__main__":
    run_safe_impedance()