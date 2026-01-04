#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 定義儲存上一次狀態的結構
struct MarkerState {
    cv::Vec3d tvec;
    tf2::Quaternion q;
    bool is_initialized = false;
};

class RobotDetectorNode : public rclcpp::Node {
public:
    RobotDetectorNode() : Node("Robot_detector_node") {
        // 1. 設定 Marker 大小 、 Slerp 插值係數
        this->declare_parameter("marker_size", 0.1); 
        marker_size_ = this->get_parameter("marker_size").as_double();

        this->declare_parameter("filter_alpha", 0.15);
        filter_alpha_ = this->get_parameter("filter_alpha").as_double();

        // 2. 訂閱相機影像
        using std::placeholders::_1;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&RobotDetectorNode::image_callback, this, _1));
        
        // 3. 訂閱校正資訊
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_self_info", 10, std::bind(&RobotDetectorNode::info_callback, this, _1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 4. 初始化 Aruco
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();

        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        detector_params_->cornerRefinementWinSize = 9;
        detector_params_->cornerRefinementMaxIterations = 90;
        detector_params_->cornerRefinementMinAccuracy = 0.03;

        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 45;
        detector_params_->adaptiveThreshWinSizeStep = 4;
        detector_params_->adaptiveThreshConstant = 7;

        detector_params_->minMarkerPerimeterRate = 0.03;
        detector_params_->maxMarkerPerimeterRate = 4.0;
        detector_params_->minCornerDistanceRate = 0.03;
        detector_params_->minMarkerDistanceRate = 0.05;

        // 5. 預設內參
        this->declare_parameter("camera_matrix", std::vector<double>(9, 0.0));
        this->declare_parameter("dist_coeffs", std::vector<double>(5, 0.0));

        std::vector<double> k_vec = this->get_parameter("camera_matrix").as_double_array();
        std::vector<double> d_vec = this->get_parameter("dist_coeffs").as_double_array();

        cam_matrix_ = cv::Mat(3, 3, CV_64F, k_vec.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, d_vec.data()).clone();

        // 6. 設定目標 ID
        target_ids_ = {2, 6};

        RCLCPP_INFO(this->get_logger(), "Robot Detector Node started.");
    }

private:
    // Slerp 濾波函式
    tf2::Quaternion slerp_quat(const tf2::Quaternion &q1_in, const tf2::Quaternion &q2_in, double t) {
        tf2::Quaternion q1 = q1_in;
        tf2::Quaternion q2 = q2_in;

        double dot = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();

        if (dot < 0.0) {
            q2 = tf2::Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
            dot = -dot;
        }
        const double DOT_THRESH = 0.9995;
        if (dot > DOT_THRESH) {
            tf2::Quaternion res(
                q1.x() + t*(q2.x() - q1.x()),
                q1.y() + t*(q2.y() - q1.y()),
                q1.z() + t*(q2.z() - q1.z()),
                q1.w() + t*(q2.w() - q1.w())
            );
            res.normalize();
            return res;
        }
        double theta_0 = std::acos(dot);
        double sin_theta_0 = std::sin(theta_0);
        double s0 = std::sin((1.0 - t) * theta_0) / sin_theta_0;
        double s1 = std::sin(t * theta_0) / sin_theta_0;
        tf2::Quaternion res(
            s0 * q1.x() + s1 * q2.x(),
            s0 * q1.y() + s1 * q2.y(),
            s0 * q1.z() + s1 * q2.z(),
            s0 * q1.w() + s1 * q2.w()
        );
        res.normalize();
        return res;
    }

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // 更新內參矩陣 K
        cam_matrix_ = (cv::Mat_<double>(3, 3) << 
            msg->k[0], msg->k[1], msg->k[2],
            msg->k[3], msg->k[4], msg->k[5],
            msg->k[6], msg->k[7], msg->k[8]);

        // 更新畸變係數 D
        dist_coeffs_ = cv::Mat(msg->d).clone();
        
        RCLCPP_INFO_ONCE(this->get_logger(), "Camera parameters updated from topic.");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat debug_frame = cv_ptr->image.clone();

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_);

        if (ids.empty()) {
            cv::imshow("Aruco Debug", debug_frame);
            cv::waitKey(1);
            return;
        }
        cv::aruco::drawDetectedMarkers(debug_frame, corners, ids);
        // 定義 3D 空間中 Marker 的四個角點座標
        std::vector<cv::Point3f> objPoints = {
            cv::Point3f(-marker_size_/2,  marker_size_/2, 0),
            cv::Point3f( marker_size_/2,  marker_size_/2, 0),
            cv::Point3f( marker_size_/2, -marker_size_/2, 0),
            cv::Point3f(-marker_size_/2, -marker_size_/2, 0)
        };

        for (size_t i = 0; i < ids.size(); i++) {
            int id = ids[i];
            // 過濾 ID
            if (std::find(target_ids_.begin(), target_ids_.end(), id) != target_ids_.end()) {
                cv::Vec3d rvec, tvec;
                if (cv::solvePnP(objPoints, corners[i], cam_matrix_, dist_coeffs_, rvec, tvec)) {
                    cv::drawFrameAxes(debug_frame, cam_matrix_, dist_coeffs_, rvec, tvec, marker_size_ * 0.5f);
                    // 轉換目前的 Rotation 為四元數
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    tf2::Matrix3x3 tf2_R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
                    tf2::Quaternion q_current;
                    tf2_R.getRotation(q_current);

                    // --- 執行濾波程序 ---
                    if (filter_states_.find(id) == filter_states_.end()) {
                        filter_states_[id] = {tvec, q_current, true};
                    } else {
                        // 平移線性濾波
                        filter_states_[id].tvec = filter_alpha_ * tvec + (1.0 - filter_alpha_) * filter_states_[id].tvec;
                        // 旋轉 Slerp 濾波
                        filter_states_[id].q = slerp_quat(filter_states_[id].q, q_current, filter_alpha_);
                    }

                    publish_filtered_tf(filter_states_[id], id, msg->header.stamp, msg->header.frame_id);
                }
            }
        }
        cv::imshow("Aruco Debug", debug_frame);
        cv::waitKey(1);
    }

    void publish_filtered_tf(const MarkerState &state, int id, rclcpp::Time stamp, std::string frame_id) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = "robot_marker_" + std::to_string(id);

        t.transform.translation.x = state.tvec[0];
        t.transform.translation.y = state.tvec[1];
        t.transform.translation.z = state.tvec[2];

        t.transform.rotation.x = state.q.x();
        t.transform.rotation.y = state.q.y();
        t.transform.rotation.z = state.q.z();
        t.transform.rotation.w = state.q.w();

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    cv::Mat cam_matrix_, dist_coeffs_;
    std::vector<int> target_ids_;
    double marker_size_;
    double filter_alpha_;
    std::mutex data_mutex_;
    std::map<int, MarkerState> filter_states_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDetectorNode>());
    rclcpp::shutdown();
    return 0;
}