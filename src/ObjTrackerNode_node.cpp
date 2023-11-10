#include "obj_tracker/ObjTrackerNode_node.hpp"

#include "obj_tracker/tracker.hpp"

// For _1
using namespace std::placeholders;

cv::Point3f pose_to_cv(const geometry_msgs::msg::Pose& point) {
    return cv::Point3f{(float)point.position.x, (float)point.position.y, (float)point.position.z};
}

geometry_msgs::msg::Pose mat_to_pose(const cv::Mat& state) {
    geometry_msgs::msg::Pose pose{};
    pose.position.x = state.at<float>(0);
    pose.position.y = state.at<float>(1);
    pose.position.z = state.at<float>(2);

    return pose;
}

ObjTrackerNode::ObjTrackerNode(const rclcpp::NodeOptions& options) : Node("ObjTrackerNode", options) {
    // Pub Sub
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/object_poses", 10, std::bind(&ObjTrackerNode::pose_cb, this, _1));
    this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/tracks", 1);
}

void ObjTrackerNode::pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (msg->poses.empty()) return;

    static Tracker tracker{0, pose_to_cv(msg->poses[0])};

    // Predict
    rclcpp::Time t = msg->header.stamp;
    tracker.predict(t.seconds());

    // Correct
    auto state = tracker.correct(pose_to_cv(msg->poses[0]));

    // Create filtered poses to publish
    auto pose = mat_to_pose(state);
    geometry_msgs::msg::PoseArray filtered_arr{};
    filtered_arr.poses.push_back(pose);

    filtered_arr.header = msg->header;

    this->pose_pub->publish(filtered_arr);
}
