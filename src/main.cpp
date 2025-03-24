// src/main.cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include "mode.hpp"

using DrawSModeExecutorNode = px4_ros2::NodeWithModeExecutor<DrawSModeExecutor, DrawSFlightMode>;

static const std::string kNodeName = "draw_s_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawSModeExecutorNode>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}