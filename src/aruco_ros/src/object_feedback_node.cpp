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
    ObjectFeedBackNode() : Node("aruco_detector_node")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&ObjectFeedBackNode::image_callback, this, _1));

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_params_->adaptiveThreshWinSizeMin = 3;
        detector_params_->adaptiveThreshWinSizeMax = 11;
        detector_params_->adaptiveThreshWinSizeStep = 8;
        detector_params_->adaptiveThreshConstant = 4;
        detector_params_->minMarkerPerimeterRate = 0.002;
        detector_params_->maxMarkerPerimeterRate = 0.5;
        detector_params_->polygonalApproxAccuracyRate = 0.07;
        detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        detector_params_->cornerRefinementWinSize = 2;
        detector_params_->cornerRefinementMaxIterations = 30;
        detector_params_->cornerRefinementMinAccuracy = 0.05;

        // use ros2 topic echo /camera/camera_info to get camera intrinsics
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            605.9448852539062, 0.0, 314.1728515625,
            0.0, 605.9448852539062, 247.0227813720703,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        RCLCPP_INFO(this->get_logger(), "Aruco solvePnPRansac version started.");
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
        if (ids.empty()) {
            cv::imshow("Aruco Detection", frame);
            cv::waitKey(1);
            return;
        }
        for (size_t i = 0; i < ids.size(); i++) {
            if(ids[i] != 42) continue; // only process marker ID 42
            cv::Point2f center(0, 0);
            for (int j = 0; j < 4; j++) {
                center.x += corners[i][j].x;
                center.y += corners[i][j].y;
            }
            center.x /= 4.0;
            center.y /= 4.0;
            float left_boundary  = frame.cols / 3.0f;
            float right_boundary = 2.0f * frame.cols / 3.0f;

            geometry_msgs::msg::Twist cmd;
            if (center.x < left_boundary) {
                RCLCPP_INFO(this->get_logger(), "Marker %d on LEFT → move +Y", ids[i]);
                // TODO: publish +Y velocity command
                cmd.linear.y = +0.2;
            }
            else if (center.x > right_boundary) {
                RCLCPP_INFO(this->get_logger(), "Marker %d on RIGHT → move -Y", ids[i]);
                // TODO: publish -Y velocity command
                cmd.linear.y = -0.2;
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Marker %d at CENTER → Y stop", ids[i]);
                // TODO: publish zero velocity
                cmd.linear.y = 0.0;
            }
            velocity_publisher_->publish(cmd);

            RCLCPP_INFO(this->get_logger(),
                        "Marker %d center: (%.1f, %.1f)", ids[i], center.x, center.y);
            cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
        }


        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        RCLCPP_INFO(this->get_logger(), "Detected %zu markers.", ids.size());

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
