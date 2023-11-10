#include "obj_tracker/ObjTrackerNode_node.hpp"

#include "obj_tracker/tracker.hpp"

// For _1
using namespace std::placeholders;

cv::Point3f pose_to_cv(const geometry_msgs::msg::Pose& point) {
    return cv::Point3f{(float)point.position.x, (float)point.position.y, (float)point.position.z};
}

geometry_msgs::msg::Pose cv_to_pose(const cv::Point3f& point) {
    geometry_msgs::msg::Pose p{};
    p.position.x = point.x;
    p.position.y = point.y;
    p.position.z = point.z;
    return p;
}

ObjTrackerNode::ObjTrackerNode(const rclcpp::NodeOptions& options)
    : Node("ObjTrackerNode", options), mot(10, 0.02, *this) {
    // Pub Sub
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/object_poses", 10, std::bind(&ObjTrackerNode::pose_cb, this, _1));
    this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/tracks", 1);
}

void ObjTrackerNode::pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) return;

    // Convert to opencv types
    rclcpp::Time stamp = msg->header.stamp;
    std::vector<cv::Point3f> detections;
    for (auto& pose : msg->poses) {
        detections.push_back(pose_to_cv(pose));
    }

    // Filter poses
    auto filtered = this->mot.filter(detections, stamp.seconds());

    // Convert from cv to ros
    geometry_msgs::msg::PoseArray filtered_arr{};
    for (auto& [id, point] : filtered) {
        RCLCPP_INFO(this->get_logger(), "id: %lu pose: %f %f %f", id, point.x, point.y, point.z);
        filtered_arr.poses.push_back(cv_to_pose(point));
    }

    filtered_arr.header = msg->header;
    this->pose_pub->publish(filtered_arr);
}
