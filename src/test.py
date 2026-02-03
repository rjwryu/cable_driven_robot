import can
import struct
import time
import sys
import os
import math
import termios
import tty
import select

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = 'can0'
INTERFACE = 'socketcan'

# --- CALIBRATED PHYSICS (Jan 31 Gold Standard) ---
TORQUE_CONSTANT = 0.59    
FRICTION_INTERCEPT = 0.1
FRICTION_SLOPE = 0.05     

# --- ROBOT PHYSICAL PROPERTIES ---
ARM_MASS_KG = 0.41        
COM_RADIUS_M = 0.17162      

SENSOR_CALIBRATION_RATIO = 1
SAFE_CURRENT_LIMIT = 5.0  

# --- TUNING ---
Kp_HOLD = 1.5   # Stiffness (Spring)
Kd_HOLD = 0.1   # Damping (Friction)

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
    # Drain buffer to get NEWEST message
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            break
        if msg.arbitration_id == REPLY_ID:
            latest_msg = msg
            
    if latest_msg:
        raw_current = struct.unpack('<h', latest_msg.data[2:4])[0]
        raw_speed = struct.unpack('<h', latest_msg.data[4:6])[0]
        raw_pos = struct.unpack('<H', latest_msg.data[6:8])[0] 
        return raw_pos, raw_speed * 1.0, (raw_current * 0.01) * SENSOR_CALIBRATION_RATIO
    return None, None, None

# --- KEYBOARD INPUT HELPERS ---
def is_key_pressed():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def get_key():
    return sys.stdin.read(1)

def run_ui_controller():
    print("--- GRAVITY COMPENSATED CONTROLLER ---")
    print(" [W/S]   : Nudge Target Angle +/- 5 deg")
    print(" [SPACE] : Toggle FLOAT vs HOLD Mode")
    print(" [Q]     : QUIT")
    
    reset_can_interface()
    try:
        bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=1000000)
    except Exception as e:
        print(f"CAN Error: {e}")
        return

    unwrapper = MotorUnwrapper()
    print("Searching for motor... (Make sure power is ON)")

    raw_p = None
    while raw_p is None:
        send_torque(bus, 0)
        time.sleep(0.01)
        raw_p, _, _ = get_latest_state(bus)
        
    # ZEROING
    start_abs = unwrapper.update(raw_p)
    zero_offset = start_abs
    print("✅ Motor Found & Zeroed Vertical.")
    time.sleep(1)
    
    # CONTROL STATE
    target_angle = 0.0
    stiffness_mode = True # True = HOLD, False = FLOAT
    
    # Setup Terminal
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while True:
            # 1. READ INPUT
            if is_key_pressed():
                k = get_key()
                if k == 'q': 
                    break
                elif k == 'w': 
                    target_angle += 5.0
                elif k == 's': 
                    target_angle -= 5.0
                elif k == ' ': 
                    stiffness_mode = not stiffness_mode
                    # --- BUMPLESS TRANSFER FIX ---
                    # When engaging hold, set target to CURRENT position
                    # so it doesn't snap back to old target.
                    if stiffness_mode and 'current_pos' in locals():
                        target_angle = current_pos
            
            # 2. READ SENSORS
            raw_p, raw_v, meas_i = get_latest_state(bus)
            
            if raw_p is not None:
                current_pos = unwrapper.update(raw_p) - zero_offset
                
                # 3. PHYSICS ENGINE
                # A. Gravity (Always On)
                grav_torque = ARM_MASS_KG * 9.81 * COM_RADIUS_M * math.sin(math.radians(current_pos))
                i_grav = grav_torque / TORQUE_CONSTANT
                
                # B. Friction (Always On - Load Scaled)
                i_load = abs(i_grav)
                fric_val = FRICTION_INTERCEPT + (FRICTION_SLOPE * i_load)
                
                # C. Impedance (Only in Hold Mode)
                error = current_pos - target_angle
                if stiffness_mode:
                    i_pid = - (Kp_HOLD * error) - (Kd_HOLD * raw_v)
                    mode_str = "🔒 HOLD"
                    
                    # Friction helps PID
                    if abs(error) > 0.5:
                        i_fric = fric_val if error < 0 else -fric_val
                    else:
                        i_fric = 0.0
                else:
                    i_pid = 0.0
                    mode_str = "☁️ FLOAT"
                    # Friction opposes velocity (Damping feel)
                    if abs(raw_v) > 5.0:
                        i_fric = fric_val if raw_v > 0 else -fric_val
                    else:
                        i_fric = 0.0

                total_current = i_pid + i_grav + i_fric
                send_torque(bus, total_current)

                # 4. DASHBOARD UI
                # \r returns to start of line. \033[K clears the line.
                ui_str = (
                    f"\r{mode_str} | "
                    f"Tgt: {target_angle:5.1f}° | "
                    f"Act: {current_pos:5.1f}° | "
                    f"Err: {error:5.1f}° | "
                    f"Trq: {total_current:5.2f} A "
                    f"(Grav: {i_grav:4.2f})"
                )
                sys.stdout.write(ui_str + "\033[K")
                sys.stdout.flush()

            time.sleep(0.002)

    except KeyboardInterrupt:
        pass
    finally:
        send_torque(bus, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n\n🛑 STOPPED.")

if __name__ == "__main__":
    run_ui_controller()