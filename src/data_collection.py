import can
import struct
import time
import sys
import os
import math
import csv
from datetime import datetime

# --- CONFIGURATION ---
MOTOR_ID = 0x142
REPLY_ID = 0x242
CHANNEL = 'can0'
INTERFACE = 'socketcan'

# --- PHYSICS CONSTANTS (Matches gravity_compensation.py) ---
TORQUE_CONSTANT = 0.59       # Nm/A
FRICTION_INTERCEPT = 0.10    # Base Stiction
FRICTION_SLOPE = 0.05        # Load Scaling

# --- ROBOT PROPERTIES ---
ARM_MASS_KG = 0.41           
COM_RADIUS_M = 0.17162       
SENSOR_CALIBRATION_RATIO = 1 

# --- CONTROLLER GAINS ---
Kp_IMP = 0.4   # A/deg
Kd_IMP = 0.008 # A/(deg/s)

# --- SAFETY LIMITS ---
MAX_POS_ERROR = 120.0
SAFE_CURRENT_LIMIT = 5.0
MAX_VELOCITY_LIMIT = 3000.0

class ExperimentLogger:
    def __init__(self, mode_name, test_name):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"log_{mode_name}_{test_name}_{timestamp}.csv"
        
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Time_s", 
                "Target_Deg", "Actual_Deg", "Error_Deg", 
                "Velocity_DegS", 
                "Cmd_Current_A", "Meas_Current_A", 
                "Temp_C",
                "Mode", "Test_Phase"
            ])
        print(f"📄 Logging to: {self.filename}")

    def log(self, t, tgt, act, vel, cmd, meas, temp, mode, phase):
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{t:.4f}", 
                f"{tgt:.2f}", f"{act:.2f}", f"{act-tgt:.2f}", 
                f"{vel:.2f}", 
                f"{cmd:.3f}", f"{meas:.3f}",
                f"{temp:.3f}",
                mode, phase
            ])

class TrajectoryGenerator:
    def __init__(self, test_id):
        self.test_id = test_id
        self.hold_duration = 8.0 
        
        self.static_targets = [
            0, 30, 60, 90,    # Lifting Up
            60, 30, 0,        # Lowering / Crossing Zero
            -30, -60, -90,    # Dropping other side
            -60, -30, 0       # Return to home
        ]
        self.repeat_targets = [0, 90] * 10 

    def get_target(self, t_elapsed):
        if self.test_id == '3':
            return 0.0, "Disturbance_Hold"

        if self.test_id == '1':
            step_idx = int(t_elapsed / self.hold_duration)
            if step_idx < len(self.static_targets):
                return self.static_targets[step_idx], f"Static_{self.static_targets[step_idx]}"
            else:
                return 0.0, "Done"

        if self.test_id == '4':
            step_idx = int(t_elapsed / 4.0)
            if step_idx < len(self.repeat_targets):
                return self.repeat_targets[step_idx], f"Rep_{step_idx}"
            else:
                return 0.0, "Done"
        
        return 0.0, "Idle"

class MotorInterface:
    def __init__(self):
        self.unwrapper = MotorUnwrapper()
        self.bus = None
        self.zero_offset = 0.0

    def connect(self):
        os.system(f"sudo ip link set {CHANNEL} down 2>/dev/null")
        os.system(f"sudo ip link set {CHANNEL} up type can bitrate 1000000 2>/dev/null")
        try:
            self.bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=1000000)
            print("✅ CAN Bus Connected")
        except Exception as e:
            print(f"❌ CAN Error: {e}")
            sys.exit(1)

    def zero_motor(self):
        print("Searching for motor... (Hold Vertical)")
        raw_p = None
        while raw_p is None:
            self.send_torque(0)
            time.sleep(0.01)
            raw_p, _, _ ,_= self.get_state()
            if raw_p is None: time.sleep(0.1)
        
        self.zero_offset = self.unwrapper.update(raw_p)
        print("✅ Zeroed at current position.")

    def send_torque(self, current):
        current = max(-SAFE_CURRENT_LIMIT, min(SAFE_CURRENT_LIMIT, current))
        data = struct.pack('<Bxxxhxx', 0xA1, int(current / 0.01))
        try:
            self.bus.send(can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False))
        except:
            pass

    def get_state(self):
        # Optimized drain loop from your working files
        latest_msg = None
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None: break
            if msg.arbitration_id == REPLY_ID:
                latest_msg = msg
        
        if latest_msg:
            temp_c = latest_msg.data[1]
            raw_i = struct.unpack('<h', latest_msg.data[2:4])[0]
            raw_v = struct.unpack('<h', latest_msg.data[4:6])[0]
            raw_p = struct.unpack('<H', latest_msg.data[6:8])[0]
            return raw_p, raw_v * 1.0, (raw_i * 0.01) * SENSOR_CALIBRATION_RATIO, temp_c
        return None, None, None, None

class MotorUnwrapper:
    def __init__(self):
        self.last_raw = None
        self.turns = 0
    def update(self, raw_pos):
        if self.last_raw is None: self.last_raw = raw_pos
        diff = raw_pos - self.last_raw
        if diff < -8192: self.turns += 1
        elif diff > 8192: self.turns -= 1
        self.last_raw = raw_pos
        base_angle = (self.turns * 360.0) + ((raw_pos / 16384.0) * 360.0)
        return base_angle * (360 / 8)

# --- MAIN EXPERIMENT LOOP ---
def run_experiment():
    print("\n=== EXPERIMENT CONFIGURATION ===")
    print("Select CONTROL MODE:")
    print("  [A] Impedance Only (No Physics)")
    print("  [B] Impedance + Gravity + Friction (Physics)")
    mode_sel = input("Selection (A/B): ").upper()
    
    print("\nSelect TEST PROTOCOL:")
    print("  [1] Static Accuracy (-30...30)")
    print("  [3] Disturbance Rejection (Hold 0)")
    print("  [4] Repeatability (0-30-0)")
    test_sel = input("Selection (1/3/4): ")
    
    if mode_sel not in ['A', 'B']: mode_sel = 'A'
    
    motor = MotorInterface()
    motor.connect()
    motor.zero_motor()
    
    traj = TrajectoryGenerator(test_sel)
    logger = ExperimentLogger(f"Mode{mode_sel}", f"Test{test_sel}")
    
    print(f"\n🚀 STARTING TEST {test_sel} in MODE {mode_sel}...")
    print("Press CTRL+C to Abort safely.")
    time.sleep(1)

    # Initialize control variable
    u_total = 0.0

    try:
        start_t = time.time()
        while True:
            t_now = time.time() - start_t

            # 1. SEND COMMAND FIRST (Triggers the motor reply)
            motor.send_torque(u_total)
            
            # 2. READ STATE SECOND
            raw_p, raw_v, meas_i, temp_c = motor.get_state()
            
            # If we missed a packet, skip calculation but keep looping to try again
            if raw_p is None: 
                continue 
            
            curr_pos = motor.unwrapper.update(raw_p) - motor.zero_offset
            
            # 3. Get Target & Check Exit
            target_pos, phase = traj.get_target(t_now)
            if phase == "Done":
                print("\n✅ Test Complete.")
                break
                
            # 4. Calculate Error
            error = curr_pos - target_pos

            # --- SAFETY CHECKS ---
            if abs(error) > MAX_POS_ERROR:
                print(f"\n❌ SAFETY STOP: Position Error {error:.1f}°")
                break
            if abs(raw_v) > MAX_VELOCITY_LIMIT:
                print(f"\n❌ SAFETY STOP: Velocity {raw_v:.1f}°/s")
                break
            
            # --- CONTROLLER IMPLEMENTATION ---
            
            # Term 1: Impedance
            u_imp = - (Kp_IMP * error) - (Kd_IMP * raw_v)
            
            # Term 2: Physics (Gravity + Friction)
            if mode_sel == 'B':
                # Gravity
                grav_torque = ARM_MASS_KG * 9.81 * COM_RADIUS_M * math.sin(math.radians(curr_pos))
                u_grav = grav_torque / TORQUE_CONSTANT
                
                # Friction (Load Scaled)
                load_amps = abs(u_grav)
                fric_mag = FRICTION_INTERCEPT + (FRICTION_SLOPE * load_amps)
                
                # Friction Direction
                if abs(error) > 0.5:
                    u_fric = fric_mag if error < 0 else -fric_mag
                else:
                    u_fric = 0.0
            else:
                u_grav = 0.0
                u_fric = 0.0
            
            # Update u_total for the NEXT loop iteration
            u_total = u_imp + u_grav + u_fric
            
            # 5. Log & UI
            logger.log(t_now, target_pos, curr_pos, raw_v, u_total, meas_i, temp_c, mode_sel, phase)
            
            sys.stdout.write(f"\r[{mode_sel}|{phase}] Tgt:{target_pos:3.0f} Act:{curr_pos:5.1f} Err:{error:5.1f} Cur:{u_total:5.2f} Temp:{temp_c}°C")
            sys.stdout.flush()
            
            time.sleep(0.002) 

    except KeyboardInterrupt:
        print("\n⚠ Aborted by user.")
    finally:
        motor.send_torque(0)
        print("\nSaved log file.")

if __name__ == "__main__":
    run_experiment()