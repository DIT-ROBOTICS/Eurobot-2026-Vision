#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class SimaDetectorNode : public rclcpp::Node
{
public:
    SimaDetectorNode() : Node("sima_detector")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera_cb/color/image_raw", 10,
            std::bind(&SimaDetectorNode::image_callback, this, _1));
        sima_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sima_pose", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
        detector_params_ = cv::aruco::DetectorParameters::create();
    
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            916.026611328125, 0.0, 653.2020263671875,
            0.0, 913.7075805664062, 366.0958251953125,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        MARKER_LENGTH = this->declare_parameter<double>("marker_length", 0.08);
        SIMA_ID = this->declare_parameter<int>("sima_id", 0);

        RCLCPP_INFO(this->get_logger(), "Sima detector started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);

        // publish marker tfs
        if(ids.size() > 0){
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for(size_t i=0; i<ids.size(); i++){
                if(ids[i] != SIMA_ID) continue;

                tf2::Vector3 t_cm(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                cv::Mat R_mat;
                cv::Rodrigues(rvecs[i], R_mat);
                tf2::Matrix3x3 m(
                    R_mat.at<double>(0,0), R_mat.at<double>(0,1), R_mat.at<double>(0,2),
                    R_mat.at<double>(1,0), R_mat.at<double>(1,1), R_mat.at<double>(1,2),
                    R_mat.at<double>(2,0), R_mat.at<double>(2,1), R_mat.at<double>(2,2));
                tf2::Quaternion q_cm; 
                m.getRotation(q_cm);

                tf2::Transform T_cm(q_cm, t_cm);

                // --- 廣播 TF ---
                geometry_msgs::msg::TransformStamped out;
                out.header.stamp = this->get_clock()->now();
                out.header.frame_id = "camera_cb_color_optical_frame";
                out.child_frame_id = "sima_" + std::to_string(ids[i]);
                out.transform = tf2::toMsg(T_cm);
                tf_broadcaster_->sendTransform(out);

                try {
                    geometry_msgs::msg::TransformStamped map2sima_tf = tf_buffer_->lookupTransform("map", "sima_" + std::to_string(ids[i]), tf2::TimePointZero);
                    geometry_msgs::msg::PoseWithCovarianceStamped sima_pose_msg;
                    sima_pose_msg.header.stamp = this->get_clock()->now();
                    sima_pose_msg.header.frame_id = "map";
                    sima_pose_msg.pose.pose.position.x = map2sima_tf.transform.translation.x;
                    sima_pose_msg.pose.pose.position.y = map2sima_tf.transform.translation.y;
                    sima_pose_msg.pose.pose.position.z = map2sima_tf.transform.translation.z;
                    sima_pose_msg.pose.pose.orientation.x = map2sima_tf.transform.rotation.x;
                    sima_pose_msg.pose.pose.orientation.y = map2sima_tf.transform.rotation.y;
                    sima_pose_msg.pose.pose.orientation.z = map2sima_tf.transform.rotation.z;
                    sima_pose_msg.pose.pose.orientation.w = map2sima_tf.transform.rotation.w;
                    sima_pose_msg.pose.covariance[0] = 0.05 * 0.05;   // x variance
                    sima_pose_msg.pose.covariance[7] = 0.05 * 0.05;   // y variance
                    sima_pose_msg.pose.covariance[14] = 1e-6;                              // z variance (small but non-zero for 2D)
                    sima_pose_msg.pose.covariance[21] = 1e-6;                              // roll variance (small but non-zero for 2D)
                    sima_pose_msg.pose.covariance[28] = 1e-6;                              // pitch variance (small but non-zero for 2D)
                    sima_pose_msg.pose.covariance[35] = 0.05 * 0.05; // yaw variance
                    sima_publisher_->publish(sima_pose_msg);

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                    continue;
                }
            }
        }
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sima_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    double MARKER_LENGTH;
    int SIMA_ID;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimaDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
