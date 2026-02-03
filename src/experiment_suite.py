import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import time
import csv
import math

# -------- CONFIGURATION (YOUR CALIBRATED VALUES) --------
PIXEL_ZERO = 347.0   # Measured at 0 degrees (Center)
PIXEL_REF  = 395.0   # Measured at 5 degrees (Right)
REF_ANGLE  = 5.0    # The angle used for calibration

# --- MATH SETUP ---
try:
    ref_rad = math.radians(REF_ANGLE)
    SCALE_FACTOR = (PIXEL_REF - PIXEL_ZERO) / math.tan(ref_rad)
except ZeroDivisionError:
    SCALE_FACTOR = 0
    print("CALIBRATION ERROR: Scale factor is zero!")

def get_degrees_from_pixel(px):
    if SCALE_FACTOR == 0: return 0
    val = (px - PIXEL_ZERO) / SCALE_FACTOR
    return math.degrees(math.atan(val))

class ExperimentSuite(Node):
    def __init__(self):
        super().__init__('experiment_suite')
        
        # Publishers & Subscribers
        self.target_pub = self.create_publisher(Float32, '/servo/set_target_angle', 10)
        self.laser_sub = self.create_subscription(Point, '/laser/position', self.laser_callback, 10)
        
        self.current_pixel = PIXEL_ZERO
        self.data_log = [] 
        self.start_time = time.time()
        
        print(f"✅ Experiment Suite Ready.")
        print(f"   Range: 0° to +20°")
        print(f"   Calibration: 0°={PIXEL_ZERO}px, {REF_ANGLE}°={PIXEL_REF}px")

    def laser_callback(self, msg):
        self.current_pixel = msg.x

    def send_target(self, angle_deg):
        msg = Float32()
        msg.data = float(angle_deg)
        self.target_pub.publish(msg)

    # --- THE CRITICAL FIX: WAIT WHILE LISTENING ---
    def wait_and_spin(self, duration_sec):
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            # Check for new messages instantly (keeps callbacks alive)
            rclpy.spin_once(self, timeout_sec=0.01)

    def record_sample(self, target_deg):
        t = time.time() - self.start_time
        actual_deg = get_degrees_from_pixel(self.current_pixel)
        self.data_log.append([t, target_deg, actual_deg, self.current_pixel])
        return actual_deg

    def save_data(self, filename):
        print(f"Having data to {filename}...")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Target_Deg", "Actual_Deg", "Raw_Pixel"])
            writer.writerows(self.data_log)
        print(f"Data saved successfully.")

    # --- EXPERIMENT 1: HYSTERESIS SWEEP (+20 to -20) ---
    def run_hysteresis(self):
        print("\n🧪 STARTING EXPERIMENT 1: Hysteresis Sweep")
        print("Sequence: 0 -> 20 -> -20 -> 0 (1° increments)")
        
        targets = list(range(0, 21, 1)) + list(range(19, -1, -1)) #        targets = list(range(0, 21, 1)) + list(range(19, -21, -1)) + list(range(-19, 1, 1))

        
        self.start_time = time.time()
        for deg in targets:
            self.send_target(deg)
            
            # Use wait_and_spin instead of time.sleep
            self.wait_and_spin(1.5) 
            
            avg_deg = 0
            for _ in range(10):
                avg_deg += self.record_sample(deg)
                # Spin briefly between samples
                self.wait_and_spin(0.05)
            
            print(f"Target: {deg:3d}° | Actual: {avg_deg/10:.2f}° | Px: {self.current_pixel:.1f}")
            
        self.save_data("exp1_hysteresis.csv")

    # --- EXPERIMENT 2: STEP RESPONSE (0 -> 20) ---
    def run_step_response(self):
        print("\n🧪 STARTING EXPERIMENT 2: Step Response")
        self.send_target(0)
        self.wait_and_spin(3.0) # Wait to stabilize
        
        print("🚀 JUMPING to 20 degrees...")
        self.start_time = time.time()
        end_time = self.start_time + 5.0
        
        while time.time() < end_time:
            target = 0 if (time.time() - self.start_time) < 0.5 else 20
            self.send_target(target)
            self.record_sample(target)
            self.wait_and_spin(0.02) # Fast recording (50Hz)
            
        self.save_data("exp2_step_response.csv")

    # --- EXPERIMENT 3: REPEATABILITY ---
    def run_repeatability(self):
        print("\n🧪 STARTING EXPERIMENT 3: Repeatability")
        
        self.start_time = time.time()
        for i in range(20):
            # Move Out to 20
            self.send_target(20)
            self.wait_and_spin(1.5)
            self.record_sample(20)
            
            # Return Home
            self.send_target(0)
            self.wait_and_spin(1.5)
            actual = self.record_sample(0)
            print(f"Loop {i+1}/20: Return Error = {actual:.3f}°")
            
        self.save_data("exp3_repeatability.csv")

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentSuite()
    
    print("\nSelect Experiment (Range: +/- 20°):")
    print("[1] Hysteresis Loop")
    print("[2] Step Response")
    print("[3] Repeatability Test")
    choice = input("Enter number: ")
    
    try:
        if choice == '1':
            node.run_hysteresis()
        elif choice == '2':
            node.run_step_response()
        elif choice == '3':
            node.run_repeatability()
        else:
            print("Invalid selection.")
    except KeyboardInterrupt:
        print("Stopping...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()