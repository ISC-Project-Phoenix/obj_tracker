#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "obj_tracker/ObjTrackerNode_node.hpp"

TEST(ObjTrackerNode, Test1) {}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}