#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h> // Helper to convert ROS -> OpenCV
#include <opencv2/opencv.hpp>

// -------- TUNING --------
// Since your room is dark, the laser will be VERY bright (200-255).
#define BRIGHTNESS_THRESHOLD 200 

using std::placeholders::_1;

class LaserTracker : public rclcpp::Node {
public:
    LaserTracker() : Node("laser_tracker") {
        // 1. Subscribe to Camera
        // Note: We use "sensor_msgs::msg::Image"
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&LaserTracker::image_callback, this, _1));

        // 2. Publisher for Laser Coordinates
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/laser/position", 10);

        RCLCPP_INFO(this->get_logger(), "Laser Tracker Started. Waiting for video...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat img = cv_ptr->image;
            cv::Mat mask;

            // --- FIX 1: Use Red Channel instead of Grayscale ---
            // Split image into Blue, Green, Red channels
            std::vector<cv::Mat> channels;
            cv::split(img, channels);
            cv::Mat red_channel = channels[2]; // Index 2 is Red in BGR

            // --- FIX 2: Debugging the Brightness ---
            double minVal, maxVal;
            cv::minMaxLoc(red_channel, &minVal, &maxVal);
            // RCLCPP_INFO(this->get_logger(), "Max Brightness: %.0f", maxVal); 

            // --- FIX 3: Dynamic Thresholding ---
            // If the max brightness is 255 (saturated), use 250. 
            // If it's dim (e.g. 150), use 100.
            // For now, let's lower the hard threshold to catch the laser.
            int safe_threshold = 150; 
            
            // Apply threshold to the RED channel only
            cv::threshold(red_channel, mask, safe_threshold, 255, cv::THRESH_BINARY);

            // Find Contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                cv::Moments m = cv::moments(max_contour);
                if (m.m00 > 0) {
                    int cx = static_cast<int>(m.m10 / m.m00);
                    int cy = static_cast<int>(m.m01 / m.m00);

                    auto point_msg = geometry_msgs::msg::Point();
                    point_msg.x = cx;
                    point_msg.y = cy;
                    publisher_->publish(point_msg);

                    // Draw Crosshair
                    cv::circle(img, cv::Point(cx, cy), 10, cv::Scalar(0, 255, 0), 3);
                    cv::line(img, cv::Point(cx-15, cy), cv::Point(cx+15, cy), cv::Scalar(0, 255, 0), 2);
                    cv::line(img, cv::Point(cx, cy-15), cv::Point(cx, cy+15), cv::Scalar(0, 255, 0), 2);
                }
            }

            cv::imshow("Robot Vision (Red Channel)", img);
            cv::imshow("Mask (Thresholded)", mask);
            cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserTracker>());
    rclcpp::shutdown();
    return 0;
}