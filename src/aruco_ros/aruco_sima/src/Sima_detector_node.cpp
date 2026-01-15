#include <memory>
#include <vector>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// 濾波器狀態結構
struct MarkerState {
    cv::Vec3d tvec;
    tf2::Quaternion q;
    bool is_initialized = false;
};

class SimaDetectorNode : public rclcpp::Node {
public:
    SimaDetectorNode() : Node("sima_detector") {
        // 1. 設定參數
        this->declare_parameter("tag_size", 0.08); 
        marker_length_ = this->get_parameter("tag_size").as_double();

        this->declare_parameter("filter_alpha", 0.15);
        filter_alpha_ = this->get_parameter("filter_alpha").as_double();

        this->declare_parameter("sima_ids", 1);
        SIMA_ID = this->get_parameter("sima_ids").as_int();
        
        // 2. 字典設定
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
        detector_params_ = cv::aruco::DetectorParameters::create();
        // --- 角點精細化
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
        detector_params_->cornerRefinementWinSize = 7;
        detector_params_->cornerRefinementMaxIterations = 50;
        detector_params_->cornerRefinementMinAccuracy = 0.01;
        // --- 自適應閾值
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 23;
        detector_params_->adaptiveThreshWinSizeStep = 2;
        detector_params_->adaptiveThreshConstant = 9;
        // --- 尺寸過濾
        detector_params_->minMarkerPerimeterRate = 0.04;
        detector_params_->maxMarkerPerimeterRate = 4.0;
        detector_params_->minCornerDistanceRate = 0.05;
        detector_params_->minMarkerDistanceRate = 0.05;

        detector_params_->markerBorderBits = 1; //減少標籤內部格子的採樣邊距，讓它更靠近格子中心，避開模糊邊界
        detector_params_->errorCorrectionRate = 0.6; // 提高解碼時對錯誤位元的容忍（16h5 很脆弱，通常設為 0 或 1）
        detector_params_->polygonalApproxAccuracyRate = 0.05; // 增加這個值可以濾掉那些「看起來不夠方」的雜訊。
        detector_params_->aprilTagQuadDecimate = 2.0; // 數值越高越快，但極遠處的小標籤會抓不到

        // 3. 初始化通訊組件
        std::string topic_name = "sima_" + std::to_string(SIMA_ID) + "/pose/global";
        sima_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera_cb/color/image_raw", 10, std::bind(&SimaDetectorNode::image_callback, this, std::placeholders::_1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_cb/color/camera_info", 10, std::bind(&SimaDetectorNode::camera_info_callback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 4. 相機內參
        this->declare_parameter("camera_matrix", std::vector<double>(9, 0.0));
        this->declare_parameter("dist_coeffs", std::vector<double>(5, 0.0));
        std::vector<double> k_vec = this->get_parameter("camera_matrix").as_double_array();
        std::vector<double> d_vec = this->get_parameter("dist_coeffs").as_double_array();
        camera_matrix_ = cv::Mat(3, 3, CV_64F, k_vec.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, d_vec.data()).clone();

        RCLCPP_INFO(this->get_logger(), "Sima_%d Detector Node started.", SIMA_ID);
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info) {
        camera_matrix_ = (cv::Mat_<double>(3,3) << 
            info->k[0], info->k[1], info->k[2],
            info->k[3], info->k[4], info->k[5],
            info->k[6], info->k[7], info->k[8]);
        dist_coeffs_ = cv::Mat(info->d).clone();
        dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (camera_matrix_.empty() || camera_matrix_.at<double>(0,0) == 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待內參中");
            return; 
        }
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat gray;
        cv::Mat debug_frame = cv_ptr->image.clone();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(gray, gray);
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

        // cv::aruco::drawDetectedMarkers(debug_frame, corners, ids, cv::Scalar(0, 255, 0));
        // if (ids.empty()) {
        //     cv::imshow("Sima Detector", debug_frame);
        //     cv::waitKey(1);
        //     return;
        // }

        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] != SIMA_ID) continue;
            // 1. SolvePnP 得到原始位置與旋轉向量
            float s = marker_length_ / 2.0f;
            std::vector<cv::Point3f> objPoints = {{-s, s, 0}, {s, s, 0}, {s, -s, 0}, {-s, -s, 0}};
            cv::Vec3d rvec, tvec;
            if (cv::solvePnP(objPoints, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
                cv::solvePnPRefineVVS(objPoints, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);
                // 2. 轉換為四元數
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                tf2::Matrix3x3 tf2_R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                tf2::Quaternion q_current;
                tf2_R.getRotation(q_current);
                // 3. 濾波程序
                if (!state_.is_initialized) {
                    state_ = {tvec, q_current, true};
                } else {
                    state_.tvec = filter_alpha_ * tvec + (1.0 - filter_alpha_) * state_.tvec;
                    state_.q = state_.q.slerp(q_current, filter_alpha_); // SLERP 旋轉濾波
                }
                // 4. 發布 TF 與 Pose
                publish_filtered_data(state_, ids[i], msg->header.stamp);
                // cv::drawFrameAxes(debug_frame, camera_matrix_, dist_coeffs_, rvec, tvec, marker_length_ * 0.5f);
            }
        }
        // cv::imshow("Sima Detector", debug_frame);
        // cv::waitKey(1);
    }

    void publish_filtered_data(const MarkerState &state, int id, rclcpp::Time stamp) {
        // --- A. 發布 Camera -> Sima TF ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "camera_cb_color_optical_frame";
        t.child_frame_id = "sima_marker_" + std::to_string(id);
        t.transform.translation.x = state.tvec[0];
        t.transform.translation.y = state.tvec[1];
        t.transform.translation.z = state.tvec[2];
        t.transform.rotation = tf2::toMsg(state.q);
        tf_broadcaster_->sendTransform(t);

        // --- B. 查詢 Map -> Sima 並發布 Pose (Localization) ---
        try {
            // 查詢 Map 到 Marker 的轉換
            auto map2sima = tf_buffer_->lookupTransform("map", t.child_frame_id, tf2::TimePointZero);
            
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = "map";
            pose_msg.pose.pose.position.x = map2sima.transform.translation.x;
            pose_msg.pose.pose.position.y = map2sima.transform.translation.y;
            pose_msg.pose.pose.position.z = map2sima.transform.translation.z;
            pose_msg.pose.pose.orientation = map2sima.transform.rotation;

            std::cout << "Current Position -> "
            << "x: " << pose_msg.pose.pose.position.x << ", "
            << "y: " << pose_msg.pose.pose.position.y << ", "
            << "z: " << pose_msg.pose.pose.position.z << std::endl;

            // 填入協方差 (對角線代表誤差 5cm 的平方)
            for(int j=0; j<36; j++) pose_msg.pose.covariance[j] = 0.0;
            pose_msg.pose.covariance[0] = 0.0025; // X
            pose_msg.pose.covariance[7] = 0.0025; // Y
            pose_msg.pose.covariance[35] = 0.0025; // Yaw (Z-axis rotation)

            sima_publisher_->publish(pose_msg);

        } catch (tf2::TransformException &ex) {
            // 靜默跳過，代表 map->camera 尚未建立
        }
    }

    // 通訊與 OpenCV 變數
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    // 參數與濾波狀態
    int SIMA_ID;
    double marker_length_, filter_alpha_;
    MarkerState state_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimaDetectorNode>());
    rclcpp::shutdown();
    return 0;
}