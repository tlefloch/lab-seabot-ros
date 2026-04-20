#include "depth_filter/depth_filter_node.hpp"

DepthFilterNode::DepthFilterNode()
: Node("depth_filter_node")
{
  init_interfaces();
}

void DepthFilterNode::init_interfaces()
{
  pressure_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "pressure", 10, std::bind(&DepthFilterNode::pressure_callback, this, std::placeholders::_1));

  // TODO: Initialize publisher for depth pose
}

void DepthFilterNode::pressure_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  depth_filter_.filter(msg->data);
  publish_msgs();
}

void DepthFilterNode::publish_msgs()
{
  // TODO: Publish depth pose message
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthFilterNode>());
  rclcpp::shutdown();
  return 0;
}