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
#include <geometry_msgs/msg/pose_stamped.hpp>

class RobotDetectorNode : public rclcpp::Node
{
public:
    RobotDetectorNode() : Node("robot_detector")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera_cb/color/image_raw", 10,
            std::bind(&RobotDetectorNode::image_callback, this, _1));
        robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cb_camera_pose", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();
    
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            916.026611328125, 0.0, 653.2020263671875,
            0.0, 913.7075805664062, 366.0958251953125,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        MARKER_LENGTH = this->declare_parameter<double>("marker_length", 0.07);
        ROBOT_ID = this->declare_parameter<int>("robot_id", 2);

        RCLCPP_INFO(this->get_logger(), "Robot detector started.");
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
                if(ids[i] != ROBOT_ID) continue;

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
                out.child_frame_id = "vision_robot_pose";
                out.transform = tf2::toMsg(T_cm);
                tf_broadcaster_->sendTransform(out);
                try {
                    geometry_msgs::msg::TransformStamped map2robot_tf = tf_buffer_->lookupTransform("map", "vision_robot_pose", tf2::TimePointZero);
                    geometry_msgs::msg::PoseStamped robot_pose_msg;
                    robot_pose_msg.header.stamp = this->get_clock()->now();
                    robot_pose_msg.header.frame_id = "vision_robot_pose";
                    robot_pose_msg.pose.position.x = map2robot_tf.transform.translation.x;
                    robot_pose_msg.pose.position.y = map2robot_tf.transform.translation.y;
                    robot_pose_msg.pose.position.z = map2robot_tf.transform.translation.z;
                    robot_pose_msg.pose.orientation.x = map2robot_tf.transform.rotation.x;
                    robot_pose_msg.pose.orientation.y = map2robot_tf.transform.rotation.y;
                    robot_pose_msg.pose.orientation.z = map2robot_tf.transform.rotation.z;
                    robot_pose_msg.pose.orientation.w = map2robot_tf.transform.rotation.w;
                    robot_pose_publisher_->publish(robot_pose_msg);

                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                    cv::imshow("SIMA Marker", frame);
                    cv::waitKey(1);
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                    continue;
                }
            }
        }
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    double MARKER_LENGTH;
    int ROBOT_ID;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
