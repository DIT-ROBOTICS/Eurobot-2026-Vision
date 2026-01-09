#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vector>
#include <string>

class PresenceDetectionNode : public rclcpp::Node
{
public:
    PresenceDetectionNode() : Node("presence_detection")
    {
        using std::placeholders::_1;

        region_inflation_ = this->declare_parameter<double>("region_inflation", 0.1);
        
        std::vector<double> default_collection = {0.0, 1.0, 0.0, 1.0, 1.0, 2.0, 0.0, 1.0}; 
        std::vector<double> default_pantry = {2.0, 3.0, 0.0, 1.0, 3.0, 4.0, 0.0, 1.0};    

        auto raw_collection = this->declare_parameter<std::vector<double>>("collection_area", default_collection);
        auto raw_pantry = this->declare_parameter<std::vector<double>>("pantry_area", default_pantry);

        collection_regions_ = process_raw_areas(raw_collection);
        pantry_regions_ = process_raw_areas(raw_pantry);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "hazelnuts_pose", 10, std::bind(&PresenceDetectionNode::pose_callback, this, _1));

        collection_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("collection_status", 10);
        pantry_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("pantry_status", 10);

        RCLCPP_INFO(this->get_logger(), "Node started. Collection regions: %zu, Pantry regions: %zu", 
                    collection_regions_.size(), pantry_regions_.size());
    }

private:

    std::vector<std::vector<double>> process_raw_areas(const std::vector<double>& raw_data) {
        std::vector<std::vector<double>> processed;
        for (size_t i = 0; i + 3 < raw_data.size(); i += 4) {
            processed.push_back({
                raw_data[i] - region_inflation_,     // min_x
                raw_data[i+1] + region_inflation_,   // max_x
                raw_data[i+2] - region_inflation_,   // min_y
                raw_data[i+3] + region_inflation_    // max_y
            });
        }
        return processed;
    }

    void check_occupancy(const geometry_msgs::msg::PoseArray::SharedPtr& msg, 
                         const std::vector<std::vector<double>>& regions, 
                         std::vector<int16_t>& status_data) {
        status_data.assign(regions.size(), 0);
        for (const auto& pose : msg->poses) {
            for (size_t i = 0; i < regions.size(); ++i) {
                if (pose.position.x >= regions[i][0] && pose.position.x <= regions[i][1] &&
                    pose.position.y >= regions[i][2] && pose.position.y <= regions[i][3]) {
                    status_data[i] = 1;
                }
            }
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        auto col_msg = std_msgs::msg::Int16MultiArray();
        auto pan_msg = std_msgs::msg::Int16MultiArray();

        check_occupancy(msg, collection_regions_, col_msg.data);
        check_occupancy(msg, pantry_regions_, pan_msg.data);

        collection_pub_->publish(col_msg);
        pantry_pub_->publish(pan_msg);
        RCLCPP_INFO(this->get_logger(), "Published occupancy status.");
        RCLCPP_INFO(this->get_logger(), "Collection status:");
        for (const auto& status : col_msg.data) {
            RCLCPP_INFO(this->get_logger(), "%d", status);  
        }
        RCLCPP_INFO(this->get_logger(), "Pantry status:");
        for (const auto& status : pan_msg.data) {
            RCLCPP_INFO(this->get_logger(), "%d", status);  
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr collection_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pantry_pub_;

    std::vector<std::vector<double>> collection_regions_;
    std::vector<std::vector<double>> pantry_regions_;
    double region_inflation_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PresenceDetectionNode>());
    rclcpp::shutdown();
    return 0;
}