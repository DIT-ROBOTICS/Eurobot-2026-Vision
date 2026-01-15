#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 定義場地標籤資訊
struct MarkerInfo {
    int id;
    cv::Point3f pos;
};

// 定義濾波狀態
struct CameraState {
    tf2::Vector3 t;
    tf2::Quaternion q;
    bool is_initialized = false;
};

class ArucoDetectorNode : public rclcpp::Node {
public:
    ArucoDetectorNode() : Node("aruco_detector_node") {
        // 1. 設定參數
        this->declare_parameter("alpha_translation", 0.2);
        this->declare_parameter("alpha_rotation", 0.2);
        alpha_trans_ = this->get_parameter("alpha_translation").as_double();
        alpha_rot_ = this->get_parameter("alpha_rotation").as_double();

        this->declare_parameter("marker_size", 0.1);
        marker_length_ =this->get_parameter("marker_size").as_double();

        // 2. 定義場地標籤位置 (World/Map 座標)
        world_markers_ = {
            {20, {0.6f, 1.4f, 0.0f}},
            {21, {2.4f, 1.4f, 0.0f}},
            {22, {0.6f, 0.6f, 0.0f}},
            {23, {2.4f, 0.6f, 0.0f}}
        };

        // 3. 初始化通訊組件
        using std::placeholders::_1;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera_cb/color/image_raw", 10, std::bind(&ArucoDetectorNode::image_callback, this, _1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_cb/color/camera_info", 10, std::bind(&ArucoDetectorNode::camera_info_callback, this, _1));
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 4. 初始化 Aruco 偵測器
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

        RCLCPP_INFO(this->get_logger(), "Aruco Detector Node started.");
    }

private:
    // SLERP 濾波
    tf2::Quaternion slerp_quat(const tf2::Quaternion &q1_in, const tf2::Quaternion &q2_in, double t) {
        tf2::Quaternion q1 = q1_in;
        tf2::Quaternion q2 = q2_in;
        double dot = q1.dot(q2);
        if (dot < 0.0) {
            q2 = tf2::Quaternion(-q2.x(), -q2.y(), -q2.z(), -q2.w());
            dot = -dot;
        }
        return q1.slerp(q2, t);
    }

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info) {
        camera_matrix_ = (cv::Mat_<double>(3,3) << 
            info->k[0], info->k[1], info->k[2],
            info->k[3], info->k[4], info->k[5],
            info->k[6], info->k[7], info->k[8]);
        dist_coeffs_ = cv::Mat(info->d).clone();
        dist_coeffs_.convertTo(dist_coeffs_, CV_64F);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (camera_matrix_.empty() || camera_matrix_.at<double>(0,0) == 0) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0);
        clahe->apply(gray, gray);
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

        // if (ids.empty()) {
        //     cv::imshow("Aruco Detector", frame);
        //     cv::waitKey(1);
        //     return;
        // }
        // cv::aruco::drawDetectedMarkers(frame, corners, ids);

        // 收集 PnP 點對
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        for (const auto &mk : world_markers_) {
            auto it = std::find(ids.begin(), ids.end(), mk.id);
            if (it != ids.end()) {
                size_t idx = std::distance(ids.begin(), it);
                float h = marker_length_ * 0.5f;
                // 注意：這裡維持你原本定義的四個角順序
                objectPoints.push_back({mk.pos.x - h, mk.pos.y + h, mk.pos.z});
                objectPoints.push_back({mk.pos.x + h, mk.pos.y + h, mk.pos.z});
                objectPoints.push_back({mk.pos.x + h, mk.pos.y - h, mk.pos.z});
                objectPoints.push_back({mk.pos.x - h, mk.pos.y - h, mk.pos.z});
                for (int k = 0; k < 4; ++k) imagePoints.push_back(corners[idx][k]);
            }
        }
        if (objectPoints.size() >= 4) {
            cv::Vec3d rvec, tvec;
            // 使用 ITERATIVE 以相容舊版，或可改為 SQPNP
            if (cv::solvePnP(objectPoints, imagePoints, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_SQPNP)) {
                cv::solvePnPRefineVVS(objectPoints, imagePoints, camera_matrix_, dist_coeffs_, rvec, tvec);
                process_camera_localization(rvec, tvec, msg->header.stamp);
            }
        }

        // cv::imshow("Aruco Detector", frame);
        // cv::waitKey(1);
    }

    void process_camera_localization(cv::Vec3d rvec, cv::Vec3d tvec, rclcpp::Time stamp) {
        // 1. Rodrigues 轉換: camera_optical -> world
        cv::Mat R_cm;
        cv::Rodrigues(rvec, R_cm);
        cv::Mat R_mc = R_cm.t();
        cv::Mat t_mc = -R_mc * cv::Mat(tvec);

        tf2::Matrix3x3 tf2_R(R_mc.at<double>(0,0), R_mc.at<double>(0,1), R_mc.at<double>(0,2),
                             R_mc.at<double>(1,0), R_mc.at<double>(1,1), R_mc.at<double>(1,2),
                             R_mc.at<double>(2,0), R_mc.at<double>(2,1), R_mc.at<double>(2,2));
        tf2::Quaternion q_copt_w;
        tf2_R.getRotation(q_copt_w);
        tf2::Vector3 v_copt_w(t_mc.at<double>(0), t_mc.at<double>(1), t_mc.at<double>(2));

        tf2::Transform T_copt_w(q_copt_w, v_copt_w);

        // 2. 取得內部位移: camera_link -> camera_color_optical_frame
        tf2::Transform T_clink_copt;
        try {
            auto tf_msg = tf_buffer_->lookupTransform("camera_cb_link", "camera_cb_color_optical_frame", tf2::TimePointZero);
            tf2::fromMsg(tf_msg.transform, T_clink_copt);
        } catch (tf2::TransformException &ex) {
            return;
        }

        // 3. 計算 world -> camera_link
        tf2::Transform T_w_clink = T_copt_w * T_clink_copt.inverse();

        // 4. 執行濾波 (EMA + SLERP)
        tf2::Vector3 cur_t = T_w_clink.getOrigin();
        tf2::Quaternion cur_q = T_w_clink.getRotation();

        if (!state_.is_initialized) {
            state_.t = cur_t;
            state_.q = cur_q;
            state_.is_initialized = true;
        } else {
            state_.t = state_.t.lerp(cur_t, alpha_trans_);
            state_.q = slerp_quat(state_.q, cur_q, alpha_rot_);
        }

        // 5. 發布 TF: map -> camera_link
        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "camera_cb_link";
        out.transform.translation.x = state_.t.x();
        out.transform.translation.y = state_.t.y();
        out.transform.translation.z = state_.t.z() - 0.01;
        out.transform.rotation = tf2::toMsg(state_.q);

        tf_broadcaster_->sendTransform(out);
    }

    // 通訊與變數
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    std::vector<MarkerInfo> world_markers_;
    double marker_length_, alpha_trans_, alpha_rot_;
    CameraState state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}