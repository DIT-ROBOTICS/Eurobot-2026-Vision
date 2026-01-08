#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MarkerWorldLocalizer : public rclcpp::Node {
public:
    MarkerWorldLocalizer() : Node("Robot_localizer_node") {
        // 1. 設定參數
        this->declare_parameter("target_marker_id", 2);
        target_marker_frame_ = "robot_marker_" + std::to_string(this->get_parameter("target_marker_id").as_int());
        world_frame_ = "map";

        // 2. 初始化 TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 建立 Publisher 與 Subscriber
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cb_camera_pose", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "final_pose", 10, std::bind(&MarkerWorldLocalizer::odom_callback, this, std::placeholders::_1));

        // 4. 建立定時器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&MarkerWorldLocalizer::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Robot Localizer Node started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time odom_time = msg->header.stamp;
        // 簡易低通濾波
        double sample_offset = (odom_time - now).seconds();
        time_offset_.store(0.9 * time_offset_.load() + 0.1 * sample_offset);
    }

    void timer_callback() {
        geometry_msgs::msg::TransformStamped transform_stamped;

        try {
            transform_stamped = tf_buffer_->lookupTransform(world_frame_, target_marker_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return;
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        
        // 時間戳校正、填入 Pose
        rclcpp::Time original_stamp = transform_stamped.header.stamp;
        pose_msg.header.stamp = original_stamp + rclcpp::Duration::from_seconds(time_offset_.load());
        pose_msg.header.frame_id = world_frame_;
        pose_msg.pose.position.x = transform_stamped.transform.translation.x;
        pose_msg.pose.position.y = transform_stamped.transform.translation.y;
        pose_msg.pose.position.z = transform_stamped.transform.translation.z;
        pose_msg.pose.orientation = transform_stamped.transform.rotation;

        pose_pub_->publish(pose_msg);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string world_frame_;
    std::string target_marker_frame_;
    std::atomic<double> time_offset_{0.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerWorldLocalizer>());
    rclcpp::shutdown();
    return 0;
}