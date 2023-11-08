#include "obj_tracker/ObjTrackerNode_node.hpp"

// For _1
using namespace std::placeholders;

ObjTrackerNode::ObjTrackerNode(const rclcpp::NodeOptions& options) : Node("ObjTrackerNode", options) {
    // Pub Sub
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/object_poses", 10, std::bind(&ObjTrackerNode::pose_cb, this, _1));
    this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/tracks", 1);
}

void ObjTrackerNode::pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg) {}
