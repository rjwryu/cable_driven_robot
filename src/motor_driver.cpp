#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

// Linux SocketCAN headers (The standard way to talk to CAN on Linux)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <vector>

// -------- Configuration --------
#define MOTOR_ID 0x141         // Motor 1 CAN ID
#define MAX_SPEED_DPS 360      // Speed limit (degrees per second)

class MotorDriver : public rclcpp::Node {
public:
    MotorDriver() : Node("motor_driver") {
        // 1. Initialize CAN Socket
        if (setup_socket("can0") < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket! Is the adapter plugged in?");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "CAN Socket Opened");

        // 2. Enable the Motor (Sequence: Clear Faults -> Enable Torque)
        send_command(MOTOR_ID, {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // 0x81: Clear Fault
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        send_command(MOTOR_ID, {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}); // 0xA1: Enable Torque
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        RCLCPP_INFO(this->get_logger(), "Motor 0x141 Enabled & Locked");

        // 3. Create Subscriber
        // Topic: /motor/move_incremental
        // Message Type: Float32 (e.g., "5.0" means move 5 degrees positive)
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor/move_incremental", 10, 
            std::bind(&MotorDriver::move_callback, this, std::placeholders::_1));
    }

    ~MotorDriver() {
        close(sock_);
    }

private:
    int sock_; // File descriptor for the socket

    // --- Low Level CAN Functions ---
    int setup_socket(const char *ifname) {
        struct sockaddr_can addr;
        struct ifreq ifr;

        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) return -1;

        strcpy(ifr.ifr_name, ifname);
        ioctl(sock_, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            return -1;
        }
        return 0;
    }

    void send_command(int id, std::vector<uint8_t> data) {
        struct can_frame frame;
        frame.can_id = id;
        frame.can_dlc = data.size();
        for (size_t i = 0; i < data.size(); ++i) frame.data[i] = data[i];

        if (write(sock_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_WARN(this->get_logger(), "CAN Write Error!");
        }
    }

    // --- ROS Callback ---
    void move_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        float target_angle = msg->data;

        // Protocol 0xA8: Incremental Position Control
        int32_t angle_control = (int32_t)(target_angle * 100.0f * 3.0f); // * 100 for 0.01 resolution * 3 for the 3:1 cable drive transmission ratio
        uint16_t speed_control = (uint16_t)(MAX_SPEED_DPS);

        std::vector<uint8_t> payload(8);
        payload[0] = 0xA8; // Command Byte
        payload[1] = 0x00;
        payload[2] = speed_control & 0xFF;        // Speed Low
        payload[3] = (speed_control >> 8) & 0xFF; // Speed High
        payload[4] = angle_control & 0xFF;        // Angle Low
        payload[5] = (angle_control >> 8) & 0xFF;
        payload[6] = (angle_control >> 16) & 0xFF;
        payload[7] = (angle_control >> 24) & 0xFF; // Angle High

        send_command(MOTOR_ID, payload);
        RCLCPP_INFO(this->get_logger(), "Moved: %.2f degrees", target_angle);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriver>());
    rclcpp::shutdown();
    return 0;
}