#pragma once

#include "geometry_msgs/msg/pose_array.hpp"
#include "mot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ObjTrackerNode : public rclcpp::Node {
private:
    /// Publishes visualisations if true
    bool debug;

    /// Filtered poses
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub;

    /// Poses to be filtered
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;

    /// Multiple object tracker
    MOT mot;

    void calc_latency(long ms) const;

public:
    ObjTrackerNode(const rclcpp::NodeOptions& options);

    /// Pose input
    void pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg);
};
