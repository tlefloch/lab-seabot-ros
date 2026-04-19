#include "simulation/simulation_node.hpp"

SimulationNode::SimulationNode()
: Node("simulation_node")
{
  init_interfaces();

  s_.init(this->now());

  physics_simulation_timer_ = this->create_wall_timer(
    physics_simulation_dt, std::bind(&SimulationNode::physics_simulation_callback, this));

  pressure_sensor_timer_ = this->create_wall_timer(
    pressure_sensor_dt, std::bind(&SimulationNode::pressure_sensor_callback, this));

  piston_simulation_timer_ = this->create_wall_timer(
    piston_simulation_dt, std::bind(&SimulationNode::piston_simulation_callback, this));

}

void SimulationNode::init_interfaces()
{
  publisher_state_ = this->create_publisher<seabot_msgs::msg::SimulationState>("simulation_state", 10);
  publisher_pressure_ = this->create_publisher<std_msgs::msg::Float64>("pressure", 10);
  piston_setpoint_subscription_ = this->create_subscription<seabot_msgs::msg::PistonSetpoint>(
    "piston_setpoint", 10, std::bind(&SimulationNode::piston_setpoint_callback, this, std::placeholders::_1));
  manual_piston_setpoint_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "manual_piston_setpoint", 10, std::bind(&SimulationNode::manual_piston_setpoint_callback, this, std::placeholders::_1));
}

void SimulationNode::physics_simulation_callback()
{
  s_.simulate_physics(physics_simulation_dt);
  auto msg = seabot_msgs::msg::SimulationState();
  msg.time = this->now().seconds();
  msg.depth = s_.x_(0);
  msg.speed = s_.x_(1);
  msg.piston_volume = s_.x_(2);
  publisher_state_->publish(msg);
}

void SimulationNode::pressure_sensor_callback()
{
  s_.simulate_pressure_sensor();
  auto msg = std_msgs::msg::Float64();
  msg.data = s_.pressure_m_;
  publisher_pressure_->publish(msg);
}

void SimulationNode::piston_setpoint_callback(const seabot_msgs::msg::PistonSetpoint::SharedPtr msg)
{
  s_.update_piston_setpoint(msg->piston_volume);
}

void SimulationNode::piston_simulation_callback()
{
  s_.simulate_piston(this->now());
}

void SimulationNode::manual_piston_setpoint_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  s_.update_piston_setpoint(s_.x_(2) + msg->linear.x * 0.0001);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}