#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

        // 2. 初始化 TF2 監聽器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 建立 Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("cb_camera_pose", 10);

        // 4. 建立定時器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&MarkerWorldLocalizer::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Localizer Node started, tracking %s in %s", 
                    target_marker_frame_.c_str(), world_frame_.c_str());
    }

private:
    void timer_callback() {
        geometry_msgs::msg::TransformStamped transform_stamped;

        // 查詢從 world 到 marker 的最新轉換
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                world_frame_, target_marker_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return;
        }

        // 封裝成 PoseStamped
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = transform_stamped.header.stamp;
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
    rclcpp::TimerBase::SharedPtr timer_;

    std::string target_marker_frame_;
    std::string world_frame_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerWorldLocalizer>());
    rclcpp::shutdown();
    return 0;
}