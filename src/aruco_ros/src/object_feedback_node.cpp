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

const double MARKER_LENGTH = 0.1; // meter

class ObjectFeedBackNode : public rclcpp::Node
{
public:
    ObjectFeedBackNode() : Node("object_feedback_node")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ObjectFeedBackNode::image_callback, this, _1));
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 11;
        detector_params_->adaptiveThreshWinSizeStep = 8;
        detector_params_->adaptiveThreshConstant = 4;
        detector_params_->minMarkerPerimeterRate = 0.002;
        detector_params_->maxMarkerPerimeterRate = 0.5;
        detector_params_->polygonalApproxAccuracyRate = 0.07;
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        detector_params_->cornerRefinementWinSize = 2;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.05;

        // use ros2 topic echo /camera/camera_info to get camera intrinsics 
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            605.9448852539062, 0.0, 314.1728515625,
            0.0, 605.9448852539062, 247.0227813720703,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        RCLCPP_INFO(this->get_logger(), "Aruco started.");
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
        rclcpp::Time now = this->now();
        const double timeout_sec = 0.3;
        if (ids.empty()) {
            if ((now - last_seen_time_).seconds() > timeout_sec) {
                // 超過 timeout 才真的停車
                geometry_msgs::msg::Twist cmd;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.angular.z = 0.0;
                velocity_publisher_->publish(cmd);
            }
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }
        for (size_t i = 0; i < ids.size(); i++) {
            if(ids[i] != 0) continue;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                std::vector<std::vector<cv::Point2f>>{corners[i]},
                MARKER_LENGTH, camera_matrix_, dist_coeffs_,
                rvecs, tvecs);
            cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_,
                                rvecs[0], tvecs[0], MARKER_LENGTH * 0.5f);
            cv::Mat R;
            cv::Rodrigues(rvecs[0], R);

            double yaw = std::atan2(R.at<double>(1,0), R.at<double>(0,0));

            geometry_msgs::msg::Twist cmd;
            double k_rot = 1.0;
            double k_lin = 0.002;
            cmd.angular.z = -k_rot * yaw;
            cv::Point2f center(0, 0);
            for (int j = 0; j < 4; j++) {
                center.x += corners[i][j].x;
                center.y += corners[i][j].y;
            }
            center.x /= 4.0;
            center.y /= 4.0;
            float left_boundary  = 3.0f * frame.cols / 7.0f;
            float right_boundary = 4.0f * frame.cols / 7.0f;
            float top_boundary = 18.0f * frame.rows / 20.0f;
            float bottom_boundary = frame.rows;

            if (center.x < left_boundary || center.x > right_boundary) {
                RCLCPP_INFO(this->get_logger(), "Marker %d on LEFT → move +Y", ids[i]);
                cmd.linear.y = -k_lin*(center.x - frame.cols/2);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Marker %d at CENTER → Y stop", ids[i]);
                cmd.linear.y = 0.0;
            }
            if (center.y < top_boundary) {
                RCLCPP_INFO(this->get_logger(), "Marker %d on TOP → move -X", ids[i]);
                cmd.linear.x = -k_lin*(center.y - (top_boundary+bottom_boundary)/2);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Marker %d at CENTER → X stop", ids[i]);
                cmd.linear.x = 0.0;
            }
            velocity_publisher_->publish(cmd);

            RCLCPP_INFO(this->get_logger(),
                        "Marker %d center: (%.1f, %.1f)", ids[i], center.x, center.y);
            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
        }


        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        RCLCPP_INFO(this->get_logger(), "Detected %zu markers.", ids.size());

        last_seen_time_ = now;
        cv::imshow("Aruco Detection", frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;
    rclcpp::Time last_seen_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFeedBackNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
