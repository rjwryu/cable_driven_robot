#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath> // For std::abs


const bool OPEN_LOOP_MODE = false; 


#define PIXEL_ZERO  347.0  // Pixel value at 0 degrees
#define PIXEL_25    395.0  // Pixel value at 5 degrees

const double PIXELS_PER_DEGREE = (PIXEL_25 - PIXEL_ZERO) / 5.0;

// Tuning
#define KP 0.08             // Gain: Convert Pixels -> Degrees
#define DEADBAND 0.5       // Stop if error is small

class VisualServo : public rclcpp::Node {
public:
    VisualServo() : Node("visual_servo") {
        // 1. Subscribe to the "Eyes" (Laser Tracker)
        laser_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/laser/position", 10, std::bind(&VisualServo::track_callback, this, std::placeholders::_1));

        // 2. Subscribe to the "Boss" (Experiment Script)
        // This tells us WHERE to go (e.g., 0, 30, 45 degrees)
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/servo/set_target_angle", 10, std::bind(&VisualServo::target_callback, this, std::placeholders::_1));

        // 3. Publish to the "Hands" (Motor Driver)
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor/move_incremental", 10);
        
        current_target_angle_ = 0.0;
        last_blind_angle_ = 0.0; // To track where we think we are in blind mode

        RCLCPP_INFO(this->get_logger(), "Visual Servo Started.");
        RCLCPP_INFO(this->get_logger(), "Mode: %s", OPEN_LOOP_MODE ? "OPEN LOOP (BLIND)" : "CLOSED LOOP (SMART)");
    }

private:
    // --- CALLBACK 1: USER SETS A NEW TARGET ---
    void target_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_target_angle_ = msg->data;

        // BLIND MODE LOGIC (OPEN LOOP)
        if (OPEN_LOOP_MODE) {
            // In blind mode, we ignore the camera. 
            // We just calculate the difference and move blindly.
            float move_needed = current_target_angle_ - last_blind_angle_;
            
            auto cmd_msg = std_msgs::msg::Float32();
            cmd_msg.data = move_needed;
            motor_pub_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Blind Move: %.1f deg", move_needed);
            
            // Update our "imaginary" position
            last_blind_angle_ = current_target_angle_;
        }
    }

    // --- CALLBACK 2: CAMERA SENDS POSITION (SERVO LOOP) ---
    void track_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        //  IF IN BLIND MODE, IGNORE THE CAMERA!
        if (OPEN_LOOP_MODE) {
            return; 
        }

        // --- SMART MODE LOGIC (CLOSED LOOP) ---
        
        // 1. Convert Target Angle -> Target Pixel
        // Formula: TargetPx = ZeroPx + (Angle * Slope)
        double target_pixel = PIXEL_ZERO + (current_target_angle_ * PIXELS_PER_DEGREE);

        // 2. Calculate Error
        double current_pixel = msg->x;
        double error_px = target_pixel - current_pixel;

        // 3. Check Deadband
        if (std::abs(error_px) < DEADBAND) {
            return; 
        }

        // 4. Calculate Correction (P-Control)
        float move_cmd = error_px * KP; 

        // 5. Safety Clamp (Max 5 degrees per jump)
        if (move_cmd > 5.0) move_cmd = 5.0;
        if (move_cmd < -5.0) move_cmd = -5.0;

        // 6. Send Command
        auto cmd_msg = std_msgs::msg::Float32();
        cmd_msg.data = move_cmd;
        motor_pub_->publish(cmd_msg);
    }

    // Variables
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr laser_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_pub_;
    
    double current_target_angle_;
    double last_blind_angle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualServo>());
    rclcpp::shutdown();
    return 0;
}