#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"

class AutoCalibratorPublisher : public rclcpp::Node {
public:
    AutoCalibratorPublisher() : Node("Camera_calibrator_node") {
        // 參數設定
        this->declare_parameter("board_width", 9);
        this->declare_parameter("board_height", 11);
        this->declare_parameter("square_size", 0.019);
        this->declare_parameter("max_samples", 30); // 總共要抓幾張

        board_size_ = cv::Size(this->get_parameter("board_width").as_int(), this->get_parameter("board_height").as_int());
        square_size_ = this->get_parameter("square_size").as_double();
        max_samples_ = this->get_parameter("max_samples").as_int();

        // ROS 通訊
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&AutoCalibratorPublisher::image_callback, this, std::placeholders::_1));
        
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_self_info", 10);

        // 每 500ms 嘗試抓取一次
        sample_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&AutoCalibratorPublisher::sampling_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Auto Calibrator started. Press [SPACE] to start auto-sampling.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        current_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        current_header_ = msg->header; // 保留 header 給發布用

        display = current_frame_.clone();
        std::string status = is_calibrating_ ? "Capturing: " + std::to_string(all_image_points_.size()) + "/" + std::to_string(max_samples_) : "Press SPACE to start";
        cv::putText(display, status, cv::Point(30, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        
        cv::imshow("Auto Calibration", display);
        char key = (char)cv::waitKey(1);
        if (key == ' ') is_calibrating_ = true;
    }

    void sampling_timer_callback() {
        if (!is_calibrating_ || current_frame_.empty() || all_image_points_.size() >= (size_t)max_samples_) {
            if (is_calibrating_ && all_image_points_.size() >= (size_t)max_samples_) {
                calculate_and_publish();
                is_calibrating_ = false;
            }
            RCLCPP_INFO(this->get_logger(), "calibrate failed");
            return;
        }

        cv::Mat gray;
        cv::cvtColor(current_frame_, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;

        // 偵測角點
        bool found = cv::findChessboardCorners(gray, board_size_, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK);
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            
            all_image_points_.push_back(corners);
            std::vector<cv::Point3f> obj;
            for (int i = 0; i < board_size_.height; i++)
                for (int j = 0; j < board_size_.width; j++)
                    obj.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
            all_object_points_.push_back(obj);

            cv::drawChessboardCorners(display, board_size_, corners, found);
            cv::imshow("Auto Calibration", display);
            RCLCPP_INFO(this->get_logger(), "Sampled %ld/%d", all_image_points_.size(), max_samples_);
        }
    }

    void calculate_and_publish() {
        cv::Mat K, D;
        std::vector<cv::Mat> rvecs, tvecs;
        
        // 執行校正
        double rms = cv::calibrateCamera(all_object_points_, all_image_points_, current_frame_.size(), K, D, rvecs, tvecs);

        // 在 Terminal 顯示結果
        std::cout << "RMS 重投影誤差: " << rms << std::endl;
        std::cout << "    camera_matrix: [" 
                << K.at<double>(0,0) << ", " << K.at<double>(0,1) << ", " << K.at<double>(0,2) << ", "
                << K.at<double>(1,0) << ", " << K.at<double>(1,1) << ", " << K.at<double>(1,2) << ", "
                << K.at<double>(2,0) << ", " << K.at<double>(2,1) << ", " << K.at<double>(2,2) << "]" << std::endl;
        std::cout << "    dist_coeffs: [" 
                << D.at<double>(0) << ", " << D.at<double>(1) << ", " << D.at<double>(2) << ", "
                << D.at<double>(3) << ", " << D.at<double>(4) << "]" << std::endl;
        std::cout << "--------------------------------------------------\n" << std::endl;
    }

    cv::Mat display,current_frame_;
    std_msgs::msg::Header current_header_;
    bool is_calibrating_ = false;
    int max_samples_;
    cv::Size board_size_;
    double square_size_;
    
    std::vector<std::vector<cv::Point2f>> all_image_points_;
    std::vector<std::vector<cv::Point3f>> all_object_points_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoCalibratorPublisher>());
    rclcpp::shutdown();
    return 0;
}