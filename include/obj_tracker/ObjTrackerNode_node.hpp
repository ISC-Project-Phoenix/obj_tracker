#pragma once

#include "geometry_msgs/msg/pose_array.hpp"
#include "mot.hpp"
#include "rclcpp/rclcpp.hpp"

class ObjTrackerNode : public rclcpp::Node {
private:
    /// Filtered poses
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub;

    /// Poses to be filtered
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub;

    /// Multiple object tracker
    MOT mot;

public:
    ObjTrackerNode(const rclcpp::NodeOptions& options);

    /// Pose input
    void pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg);
};
