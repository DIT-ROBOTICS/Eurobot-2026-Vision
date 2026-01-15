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

class RelativeDetectorNode : public rclcpp::Node {
public:
    RelativeDetectorNode() : Node("Relative_detector_node") {
        // 1. 設定參數
        this->declare_parameter("field_marker_size", 0.1);
        field_length = this->get_parameter("field_marker_size").as_double();
        this->declare_parameter("robot_marker_size", 0.07);
        robot_length = this->get_parameter("robot_marker_size").as_double();
        this->declare_parameter("robot_ids", std::vector<int>{2});
        auto raw_ids = this->get_parameter("robot_ids").as_integer_array();
        robot_ids_.assign(raw_ids.begin(), raw_ids.end());

        this->declare_parameter("alpha_translation", 0.2);
        alpha_trans_ = this->get_parameter("alpha_translation").as_double();
        this->declare_parameter("alpha_rotation", 0.2);
        alpha_rot_ = this->get_parameter("alpha_rotation").as_double();
        
        this->declare_parameter("cam_name", "cam_mid");
        cam_name_id_ = this->get_parameter("cam_name").as_string();
        camera_name_prefix_ = "/camera_cb/" + cam_name_id_;

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
            camera_name_prefix_ + "/color/image_raw", 10, std::bind(&RelativeDetectorNode::image_callback, this, _1));
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_name_prefix_ + "/color/camera_info", 10, std::bind(&RelativeDetectorNode::camera_info_callback, this, _1));
        
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

        RCLCPP_INFO(this->get_logger(), "Relative Detector Node started.");
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
        // 1. -- 處理圖片並偵測Aruco --
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

        if (ids.empty()) {
            cv::imshow("Aruco Detector", frame);
            cv::waitKey(1);
            return;
        }
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        // 2. -- 分類情況 --
        int field_idx = -1;
        int robot_idx = -1;
        int field_marker_id = -1;
        int robot_marker_id = -1;

        for (size_t i = 0; i < ids.size(); ++i) {
            for (const auto &mk : world_markers_) {   //檢查場地Aruco ID
                if (ids[i] == mk.id) {
                    field_idx = i;
                    field_marker_id = ids[i];
                    break;
                }
            }
            if (std::find(robot_ids_.begin(), robot_ids_.end(), ids[i]) != robot_ids_.end()) {   //檢查機器Aruco ID
                robot_idx = i;
                robot_marker_id = ids[i];
            }
        }

        // 情況 1: 兩者皆有 (相對定位模式)
        if (field_idx != -1 && robot_idx != -1) {
            RCLCPP_INFO(this->get_logger(), "Mode 1: Relative Positioning");
            relative_localization(field_idx, robot_idx, field_marker_id, robot_marker_id, corners, msg->header.stamp);
        }
        // 情況 2: 只有場地標籤 (相機校正模式)
        else if (field_idx != -1) {
            RCLCPP_INFO(this->get_logger(), "Mode 2: Camera Calibration");
            camera_calibration(field_idx, field_marker_id, corners, msg->header.stamp);
        }
        // 情況 3: 只有大機標籤 (動態/靜態校正補償模式)
        else if (robot_idx != -1) {
            RCLCPP_INFO(this->get_logger(), "Mode 3: Robot Localization");
            known_localization(robot_idx, robot_marker_id, corners, msg->header.stamp);
        }
        cv::imshow("Aruco Detector", frame);
        cv::waitKey(1);
    }

    void relative_localization(int f_idx, int r_idx, int f_id, int r_id, const std::vector<std::vector<cv::Point2f>>& corners, rclcpp::Time stamp) {

        // 1. 分別對場地標籤與機器人標籤解 PnP
        cv::Vec3d rvec_f, tvec_f, rvec_r, tvec_r;

        float h_f = field_length * 0.5f;
        float h_r = robot_length * 0.5f;
        std::vector<cv::Point3f> obj_f = {{-h_f,h_f,0}, {h_f,h_f,0}, {h_f,-h_f,0}, {-h_f,-h_f,0}};
        std::vector<cv::Point3f> obj_r = {{-h_r,h_r,0}, {h_r,h_r,0}, {h_r,-h_r,0}, {-h_r,-h_r,0}};

        cv::solvePnP(obj_f, corners[f_idx], camera_matrix_, dist_coeffs_, rvec_f, tvec_f, false, cv::SOLVEPNP_SQPNP);
        cv::solvePnP(obj_r, corners[r_idx], camera_matrix_, dist_coeffs_, rvec_r, tvec_r, false, cv::SOLVEPNP_SQPNP);

        // 2. 轉為矩陣
        cv::Mat R_f, R_r;
        cv::Rodrigues(rvec_f, R_f);
        cv::Rodrigues(rvec_r, R_r);

        // 3. 計算 Robot 相對於 Field 的座標 (在 Field 座標系下)
        cv::Mat p_robot_in_field = R_f.t() * (cv::Mat(tvec_r) - cv::Mat(tvec_f));
        
        // 計算旋轉相對關係 (Robot 相對於 Field 的四元數)
        cv::Mat R_rel = R_f.t() * R_r; 
        
        // 4. 取得 Field 在 Map 的已知位置 (從 world_markers_ 找)
        cv::Point3f field_pos_map;
        for(const auto &mk : world_markers_) {
            if(mk.id == f_id) {
                field_pos_map = mk.pos;
                break;
            }
        }

        // 5. 計算最終位姿 (未濾波)
        tf2::Vector3 raw_t(
            field_pos_map.x + p_robot_in_field.at<double>(0),
            field_pos_map.y + p_robot_in_field.at<double>(1),
            field_pos_map.z + p_robot_in_field.at<double>(2)
        );
        // 將旋轉矩陣 R_rel 轉為 tf2::Quaternion
        tf2::Matrix3x3 tf2_R(
            R_rel.at<double>(0,0), R_rel.at<double>(0,1), R_rel.at<double>(0,2),
            R_rel.at<double>(1,0), R_rel.at<double>(1,1), R_rel.at<double>(1,2),
            R_rel.at<double>(2,0), R_rel.at<double>(2,1), R_rel.at<double>(2,2)
        );
        tf2::Quaternion raw_q;
        tf2_R.getRotation(raw_q);

        // 6. 執行濾波 (EMA + Slerp)
        if (!robot_state_.is_initialized) {
            robot_state_.t = raw_t;
            robot_state_.q = raw_q;
            robot_state_.is_initialized = true;
        } else {
            // EMA 濾波平移
            robot_state_.t = robot_state_.t.lerp(raw_t, alpha_trans_);
            // Slerp 濾波旋轉 (調用你寫的 slerp_quat 函式)
            robot_state_.q = slerp_quat(robot_state_.q, raw_q, alpha_rot_);
        }

        // 7. 發布大機 TF: map -> robot
        geometry_msgs::msg::TransformStamped robot_tf;
        robot_tf.header.stamp = stamp;
        robot_tf.header.frame_id = "map";
        robot_tf.child_frame_id = "robot_marker_" + std::to_string(r_id);
        robot_tf.transform.translation.x = robot_state_.t.x();
        robot_tf.transform.translation.y = robot_state_.t.y();
        robot_tf.transform.translation.z = robot_state_.t.z();
        robot_tf.transform.rotation = tf2::toMsg(robot_state_.q);
        
        tf_broadcaster_->sendTransform(robot_tf);
    }

    void camera_calibration(int f_idx, int f_id, const std::vector<std::vector<cv::Point2f>>& corners, rclcpp::Time stamp) {
        // --- 1. 準備 PnP 所需的數據 ---
        float h = field_length * 0.5f;
        // 取得該標籤在 Map 下的已知世界座標
        cv::Point3f field_pos_map(0, 0, 0);
        for (const auto &mk : world_markers_) {
            if (mk.id == f_id) {
                field_pos_map = mk.pos;
                break;
            }
        }

        std::vector<cv::Point3f> objectPoints = {{-h,  h, 0}, { h,  h, 0}, { h, -h, 0}, {-h, -h, 0}};

        // --- 2. 執行 SolvePnP 解出 Camera -> Field 的位姿 ---
        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(objectPoints, corners[f_idx], camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_SQPNP)) {
            return;
        }
        cv::solvePnPRefineVVS(objectPoints, corners[f_idx], camera_matrix_, dist_coeffs_, rvec, tvec);

        // --- 3. 座標轉換: 從相機光學系 (camera_optical) 轉到地圖系 (world/map) ---
        cv::Mat R_cm;
        cv::Rodrigues(rvec, R_cm);
        cv::Mat R_mc = R_cm.t();            // 相機相對於標籤的旋轉 (R_world_cam)
        cv::Mat t_mc = -R_mc * cv::Mat(tvec); // 相機相對於標籤的平移 (t_world_cam)

        // 將 OpenCV 矩陣轉為 tf2 格式
        tf2::Matrix3x3 tf2_R(R_mc.at<double>(0,0), R_mc.at<double>(0,1), R_mc.at<double>(0,2),
                            R_mc.at<double>(1,0), R_mc.at<double>(1,1), R_mc.at<double>(1,2),
                            R_mc.at<double>(2,0), R_mc.at<double>(2,1), R_mc.at<double>(2,2));
        tf2::Quaternion q_copt_w;
        tf2_R.getRotation(q_copt_w);
        
        // 計算相機光學中心在 Map 上的位置 = 標籤位置 + 相對平移
        tf2::Vector3 v_copt_w(
            field_pos_map.x + t_mc.at<double>(0),
            field_pos_map.y + t_mc.at<double>(1),
            field_pos_map.z + t_mc.at<double>(2)
        );

        tf2::Transform T_copt_w(q_copt_w, v_copt_w);

        // --- 4. 取得內部位移: camera_link -> camera_color_optical_frame ---
        tf2::Transform T_clink_copt;
        try {
            // 注意：這裡使用的 frame 名稱需與你機器人上的 TF Tree 一致
            auto tf_msg = tf_buffer_->lookupTransform(cam_name_id_ + "_link", cam_name_id_ + "_color_optical_frame", tf2::TimePointZero);
            tf2::fromMsg(tf_msg.transform, T_clink_copt);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Internal TF lookup failed: %s", ex.what());
            return; 
        }

        // 計算 map -> camera_link: T_w_clink = T_map_copt * T_copt_clink
        tf2::Transform T_w_clink = T_copt_w * T_clink_copt.inverse();

        // --- 5. 執行濾波 (EMA + SLERP) ---
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

        // --- 6. 更新全局狀態，供情況 3 (只有大機時) 使用 ---
        last_camera_pose_ = tf2::Transform(state_.q, state_.t);
        has_dynamic_calib_ = true;

        // --- 7. 發布 TF: map -> camera_cb_link ---
        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = cam_name_id_ + "_link";
        
        out.transform.translation.x = state_.t.x();
        out.transform.translation.y = state_.t.y();
        out.transform.translation.z = state_.t.z();
        out.transform.rotation = tf2::toMsg(state_.q);

        tf_broadcaster_->sendTransform(out);
    }

    void known_localization(int r_idx, int r_id, const std::vector<std::vector<cv::Point2f>>& corners, rclcpp::Time stamp) {
        // 1. 準備 Robot 標籤的 3D 模型點
        float h_r = robot_length * 0.5f;
        std::vector<cv::Point3f> objectPoints = {{-h_r,  h_r, 0}, { h_r,  h_r, 0}, { h_r, -h_r, 0}, {-h_r, -h_r, 0}};

        // 2. 執行 SolvePnP 解出 Camera_Optical -> Robot 的位姿
        cv::Vec3d rvec, tvec;
        if (!cv::solvePnP(objectPoints, corners[r_idx], camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_SQPNP)) {
            return;
        }
        cv::solvePnPRefineVVS(objectPoints, corners[r_idx], camera_matrix_, dist_coeffs_, rvec, tvec);

        // 3. 將 OpenCV 位姿轉為 tf2::Transform (T_copt_robot)
        cv::Mat R_r;
        cv::Rodrigues(rvec, R_r);
        tf2::Matrix3x3 tf2_R(
            R_r.at<double>(0,0), R_r.at<double>(0,1), R_r.at<double>(0,2),
            R_r.at<double>(1,0), R_r.at<double>(1,1), R_r.at<double>(1,2),
            R_r.at<double>(2,0), R_r.at<double>(2,1), R_r.at<double>(2,2)
        );
        tf2::Quaternion q_r;
        tf2_R.getRotation(q_r);
        tf2::Transform T_copt_robot(q_r, tf2::Vector3(tvec[0], tvec[1], tvec[2]));

        // 4. 取得相機內部的位移: camera_link -> camera_optical_frame
        tf2::Transform T_clink_copt;
        try {
            // lookupTransform 可能會因為時間差噴 warning，建議在啟動時預取或使用 TimeZero
            auto tf_msg = tf_buffer_->lookupTransform(cam_name_id_ + "_link", cam_name_id_ + "_color_optical_frame", tf2::TimePointZero);
            tf2::fromMsg(tf_msg.transform, T_clink_copt);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Mode 3 TF lookup failed: %s", ex.what());
            return; 
        }

        // 5. 計算 Map -> Robot 的最終位姿
        // 公式: T_map_robot = T_map_clink (已知的相機位姿) * T_clink_copt * T_copt_robot
        tf2::Transform T_map_robot = last_camera_pose_ * T_clink_copt * T_copt_robot;

        // 6. 執行濾波 (EMA + SLERP)
        // 這裡使用獨立於相機的 robot_state_ 結構
        tf2::Vector3 cur_t = T_map_robot.getOrigin();
        tf2::Quaternion cur_q = T_map_robot.getRotation();

        if (!robot_state_.is_initialized) {
            robot_state_.t = cur_t;
            robot_state_.q = cur_q;
            robot_state_.is_initialized = true;
        } else {
            robot_state_.t = robot_state_.t.lerp(cur_t, alpha_trans_);
            robot_state_.q = slerp_quat(robot_state_.q, cur_q, alpha_rot_);
        }

        // 7. 發布 TF: map -> robot_marker_id (依你呼叫時傳入的 ID)
        geometry_msgs::msg::TransformStamped out;
        out.header.stamp = stamp;
        out.header.frame_id = "map";
        out.child_frame_id = "robot_marker_" + std::to_string(r_id);
        
        out.transform.translation.x = robot_state_.t.x();
        out.transform.translation.y = robot_state_.t.y();
        out.transform.translation.z = robot_state_.t.z();
        out.transform.rotation = tf2::toMsg(robot_state_.q);

        tf_broadcaster_->sendTransform(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    std::vector<MarkerInfo> world_markers_;
    std::vector<int> robot_ids_;
    std::vector<int> field_ids_;
    std::string cam_name_id_;
    std::string camera_name_prefix_;
    double field_length, robot_length, alpha_trans_, alpha_rot_;
    CameraState state_;
    CameraState robot_state_;

    tf2::Transform last_camera_pose_; // map -> camera_cb_link
    bool has_dynamic_calib_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}