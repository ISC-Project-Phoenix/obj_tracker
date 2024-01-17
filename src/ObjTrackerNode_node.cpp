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
    : Node("ObjTrackerNode", options),
      mot(this->declare_parameter("max_frames_missed", 5), this->declare_parameter("max_dist", 1.0),
          this->declare_parameter("prediction_cov", 0.01f), this->declare_parameter("measure_cov", 1.0f),
          this->declare_parameter("inital_vx", -0.3f)) {
    // Random params
    this->declare_parameter("test_latency", false);
    debug = this->declare_parameter<bool>("debug", true);

    // Pub Sub
    this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/object_poses", 10, std::bind(&ObjTrackerNode::pose_cb, this, _1));
    this->pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/tracks", 1);
    this->viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);
}

void ObjTrackerNode::pose_cb(geometry_msgs::msg::PoseArray::SharedPtr msg) {
    auto start_t = std::chrono::steady_clock::now();

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
        filtered_arr.poses.push_back(cv_to_pose(point));
    }

    filtered_arr.header = msg->header;
    this->pose_pub->publish(filtered_arr);

    // publish visualization if running with debug
    if (debug) {
        visualization_msgs::msg::MarkerArray vis{};

        for (auto& [id, point] : filtered) {
            std::stringstream ss;
            ss << "id:" << std::to_string(id);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = msg->header.frame_id;
            marker.header.stamp = rclcpp::Time{0};
            marker.lifetime = rclcpp::Duration::from_seconds(0.05);
            marker.frame_locked = true;
            marker.ns = "obj_tracker";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.text = ss.str();
            marker.action = visualization_msgs::msg::Marker::MODIFY;
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z + 0.3;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            vis.markers.push_back(marker);
        }
        this->viz_pub->publish(vis);
    }

    if (this->get_parameter("test_latency").as_bool()) {
        auto delta = std::chrono::steady_clock::now() - start_t;
        auto ms = std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count();
        calc_latency(ms);
    }
}

void ObjTrackerNode::calc_latency(long ms) const {
    static std::array<uint64_t, 300> measurements;
    static uint64_t index = 0;
    measurements[index] = ms;
    index = index + 1 > measurements.size() - 1 ? 0 : index + 1;

    // Calc statistics
    double mean = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean += (double)measurements[i];
    }
    mean /= (double)index + 1;

    double mean2 = 0;
    for (uint64_t i = 0; i != index; ++i) {
        mean2 += std::pow((double)measurements[i], 2);
    }
    mean2 /= (double)index + 1;

    double std_dev = sqrt(mean2 - std::pow(mean, 2));

    RCLCPP_INFO(get_logger(), "Mean: %fms Std-dev: %fms", mean * 1e-6, std_dev * 1e-6);
}