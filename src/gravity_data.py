import can
import struct
import time
import sys
import os
import math
import csv

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = 'can0'
INTERFACE = 'socketcan'

# --- SAFETY LIMITS ---
MAX_POS_ERROR = 120.0
SAFE_CURRENT_LIMIT = 2.0
MAX_VELOCITY_LIMIT = 3000.0 

# --- TUNING ---
Kp = 0.05        
Kd = 0.0016       # Set this ONCE here
Kf = 0.6          
VEL_FILTER = 0.30 
FRICTION_RAMP_WIDTH = 8.0 

# --- SWEEP SETTINGS ---
SWEEP_AMPLITUDE = 45.0  
SWEEP_SPEED = 0.4       
DATA_FILENAME = "sweep_data.csv"

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
        return base_angle * 8.0 * 5.625

def send_torque(bus, current):
    current = max(-SAFE_CURRENT_LIMIT, min(SAFE_CURRENT_LIMIT, current))
    current_int = int(current / 0.01)
    data = struct.pack('<Bxxxhxx', 0xA1, current_int) 
    try:
        bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
    except: pass

def get_latest_state(bus):
    latest_msg = None
    while True:
        msg = bus.recv(timeout=0)
        if msg is None: break
        if msg.arbitration_id == REPLY_ID: latest_msg = msg
            
    if latest_msg:
        raw_current = struct.unpack('<h', latest_msg.data[2:4])[0] * 0.01
        raw_speed = struct.unpack('<h', latest_msg.data[4:6])[0] * 1.0
        raw_pos = struct.unpack('<H', latest_msg.data[6:8])[0] 
        return raw_pos, raw_speed, raw_current
    return None, None, None

def run_relative_sweep():
    print(f"--- RELATIVE SWEEP (Robust) ---")
    print("1. Ensure CAN bus is UP manually.")
    
    try:
        bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=1000000)
    except Exception as e:
        print(f"CAN Error: {e}")
        return

    unwrapper = MotorUnwrapper()
    print("Connecting (Wake up)...")

    raw_p = None
    while raw_p is None:
        send_torque(bus, 0) # Keep poking it
        time.sleep(0.01)
        raw_p, _, _ = get_latest_state(bus)
        if raw_p is None: time.sleep(0.1)
        else: print("\n✅ CONNECTED!")

    # --- LOCK TO CURRENT POSITION ---
    start_pos = unwrapper.update(raw_p)
    center_pos = start_pos 
    print(f"Locked Center at: {center_pos:.2f}°")
    print("Starting Sweep in 3 seconds...")
    time.sleep(3.0)
    
    filtered_vel = 0.0
    loop_count = 0
    filter_inv = 1.0 - VEL_FILTER
    
    # Store last command to keep motor alive if packet drops
    last_cmd_torque = 0.0

    # !!! INDENTATION FIXED BELOW THIS LINE !!!
    with open(DATA_FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Time", "Target", "Actual", "Velocity", "Current"])
        
        start_time = time.time()

        try:
            while (time.time() - start_time) < 15.0: 
                t = time.time() - start_time
                
                # 1. ALWAYS SEND COMMAND (Keep Alive)
                send_torque(bus, last_cmd_torque)

                # 2. Get Data
                raw_p, raw_v, _ = get_latest_state(bus)
                
                if raw_p is not None:
                    # --- NEW CALCULATION ---
                    current_pos = unwrapper.update(raw_p)
                    
                    offset = SWEEP_AMPLITUDE * math.sin(2 * math.pi * SWEEP_SPEED * t)
                    target_pos = center_pos + offset

                    scaled_vel = raw_v 
                    filtered_vel = (VEL_FILTER * scaled_vel) + (filter_inv * filtered_vel)
                    error = target_pos - current_pos
                    
                    if abs(error) > MAX_POS_ERROR:
                        print(f"\nSAFETY STOP: Error {error:.1f}°")
                        send_torque(bus, 0)
                        break
                    
                    if abs(filtered_vel) > MAX_VELOCITY_LIMIT:
                        print(f"\nSAFETY STOP: Velocity too high!")
                        send_torque(bus, 0)
                        break

                    ramp_width = FRICTION_RAMP_WIDTH
                    err_clamped = max(-ramp_width, min(ramp_width, error))
                    ratio = err_clamped / ramp_width
                    f_term = Kf * ratio 

                    target_torque = (Kp * error) - (Kd * filtered_vel) + f_term
                    
                    # Update the "Keep Alive" value
                    last_cmd_torque = target_torque
                    
                    writer.writerow([t, target_pos, current_pos, filtered_vel, target_torque])

                    loop_count += 1
                    if loop_count >= 50:
                        sys.stdout.write(f"\rTgt: {target_pos:5.1f}° | Act: {current_pos:5.1f}° | Trq: {target_torque:5.2f}")
                        sys.stdout.flush()
                        loop_count = 0
                
                time.sleep(0.002)

        except KeyboardInterrupt:
            pass

    send_torque(bus, 0)
    print(f"\nStopped. Data saved to {DATA_FILENAME}")

if __name__ == "__main__":
    run_relative_sweep()