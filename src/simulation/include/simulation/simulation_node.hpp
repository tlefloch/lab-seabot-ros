#ifndef SIMULATION_SIMULATION_NODE_HPP_
#define SIMULATION_SIMULATION_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "simulation/simulation.hpp"
#include "std_msgs/msg/float64.hpp"
#include "seabot_msgs/msg/simulation_state.hpp"
#include "seabot_msgs/msg/piston_setpoint.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode();

private:
  // Physics simulation
  rclcpp::TimerBase::SharedPtr physics_simulation_timer_;
  std::chrono::milliseconds physics_simulation_dt = std::chrono::milliseconds(10);
  rclcpp::Publisher<seabot_msgs::msg::SimulationState>::SharedPtr publisher_state_;
  void physics_simulation_callback();

  // Pressure sensor simulation
  rclcpp::TimerBase::SharedPtr pressure_sensor_timer_;
  std::chrono::milliseconds pressure_sensor_dt = std::chrono::milliseconds(200);  // Sensor update rate (5 Hz)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_pressure_;
  void pressure_sensor_callback();

  // Piston simulation
  rclcpp::TimerBase::SharedPtr piston_simulation_timer_;
  std::chrono::milliseconds piston_simulation_dt = std::chrono::milliseconds(100);  // Simulation update rate (100 Hz)
  void piston_simulation_callback();
  rclcpp::Subscription<seabot_msgs::msg::PistonSetpoint>::SharedPtr piston_setpoint_subscription_;
  void piston_setpoint_callback(const seabot_msgs::msg::PistonSetpoint::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_piston_setpoint_subscription_;
  void manual_piston_setpoint_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  Simulation s_;

  void init_interfaces();
  
  
};

#endif  // SIMULATION_SIMULATION_NODE_HPP_