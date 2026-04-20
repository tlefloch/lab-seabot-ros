#ifndef DEPTH_FILTER_DEPTH_FILTER_NODE_HPP_
#define DEPTH_FILTER_DEPTH_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "depth_filter/depth_filter.hpp"
#include "std_msgs/msg/float64.hpp"

class DepthFilterNode : public rclcpp::Node
{
public:
  DepthFilterNode();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pressure_subscription_;
  
  // TODO: Add publisher for depth pose

  DepthFilter depth_filter_;

  void init_interfaces();
  void pressure_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void publish_msgs();
};

#endif  // DEPTH_FILTER_DEPTH_FILTER_NODE_HPP_