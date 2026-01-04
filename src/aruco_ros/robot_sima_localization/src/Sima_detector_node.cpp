#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SimaDetectorNode : public rclcpp::Node {
public:
    SimaDetectorNode() : Node("Sima_detector_node") {
        // 1. 宣告與取得參數
        this->declare_parameter("tag_size", 0.08); 
        tag_size_ = this->get_parameter("tag_size").as_double();

        // 2. 初始化 AprilTag 偵測器 (C API)
        td_ = apriltag_detector_create();
        tf_family_ = tag36h11_create(); 
        apriltag_detector_add_family(td_, tf_family_);

        // 3. TF 廣播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 4. 訂閱影像
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&SimaDetectorNode::image_callback, this, std::placeholders::_1));
            
        // 5. 預設相機內參
        this->declare_parameter("camera_matrix", std::vector<double>(9, 0.0));
        this->declare_parameter("dist_coeffs", std::vector<double>(5, 0.0));

        std::vector<double> k_vec = this->get_parameter("camera_matrix").as_double_array();
        std::vector<double> d_vec = this->get_parameter("dist_coeffs").as_double_array();

        cam_matrix_ = cv::Mat(3, 3, CV_64F, k_vec.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, d_vec.data()).clone();

        RCLCPP_INFO(this->get_logger(), "SIMA Detector Node started.");
    }

    ~SimaDetectorNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_family_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

        // 建立 AprilTag 影像格式
        image_u8_t img_header = { 
            static_cast<int32_t>(gray.cols), 
            static_cast<int32_t>(gray.rows), 
            static_cast<int32_t>(gray.cols), 
            gray.data 
        };

        // 執行偵測
        zarray_t *detections = apriltag_detector_detect(td_, &img_header);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // 取得 Tag 的四個角點
            std::vector<cv::Point2f> image_points;
            for (int j = 0; j < 4; j++) {
                image_points.push_back(cv::Point2f(static_cast<float>(det->p[j][0]), 
                                                   static_cast<float>(det->p[j][1])));
            }

            // 定義物體座標系 (Tag 中心為原點)
            float s = static_cast<float>(tag_size_ / 2.0);
            std::vector<cv::Point3f> object_points = {
                cv::Point3f(-s, -s, 0.0f), 
                cv::Point3f( s, -s, 0.0f), 
                cv::Point3f( s,  s, 0.0f), 
                cv::Point3f(-s,  s, 0.0f)
            };

            cv::Vec3d rvec, tvec;
            // 使用 IPPE_SQUARE 提高平面標籤的穩定度
            cv::solvePnP(object_points, image_points, cam_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            publish_tf(rvec, tvec, det->id, msg->header.stamp);
        }
        apriltag_detections_destroy(detections);
    }

    void publish_tf(cv::Vec3d rvec, cv::Vec3d tvec, int id, rclcpp::Time stamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = "sima_marker_" + std::to_string(id);

        t.transform.translation.x = tvec[0];
        t.transform.translation.y = tvec[1];
        t.transform.translation.z = tvec[2];

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 使用 tf2 處理旋轉
        tf2::Matrix3x3 tf2_R(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );

        tf2::Quaternion q;
        tf2_R.getRotation(q);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    apriltag_detector_t *td_;
    apriltag_family_t *tf_family_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    double tag_size_;
    cv::Mat cam_matrix_, dist_coeffs_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimaDetectorNode>());
    rclcpp::shutdown();
    return 0;
}