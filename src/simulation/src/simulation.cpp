#include "simulation/simulation.hpp"

Simulation::Simulation()
{
}

void Simulation::init(rclcpp::Time current_time)
{
  abs_pressure_ = zero_pressure_;
  x_(0) = 1.0;  // Initial depth
  x_(2) = V_piston_max_;  // Initial piston volume
  last_piston_simulation_time_ = current_time;
  piston_setpoint_ = V_piston_max_;
}

void Simulation::simulate_physics(std::chrono::milliseconds dt)
{
  x_ = x_ + f() * dt.count() * 1e-3;  // Multiply by dt in seconds

  // Update absolute pressure based on depth
  abs_pressure_ = zero_pressure_ + std::max(0.0,(density_ * g_physics_ * (-x_(0)))) * 1e-5;
}

void Simulation::simulate_pressure_sensor()
{
  pressure_m_ = abs_pressure_ + pressure_sensor_dist_(generator_); // in Bar
}

void Simulation::update_piston_setpoint(double piston_volume)
{
  piston_setpoint_ = std::clamp(piston_volume, 0.0, V_piston_max_);
}

void Simulation::simulate_piston(rclcpp::Time current_time)
{
  if (last_piston_simulation_time_.seconds() == 0) {
    last_piston_simulation_time_ = current_time;  // Initialize last update time on first call
  }
  // Limit piston volume to maximum
  double e = piston_setpoint_ - x_(2);  // Error between setpoint and current piston volume
  double sign_e = e >= 0 ? 1.0 : -1.0;  // Sign of the error
  double max_volume_change = piston_max_flow_rate_ * (current_time - last_piston_simulation_time_).seconds();  // Maximum volume change based on flow rate and time step

  // Update state based on piston volume
  x_(2) = x_(2) + std::min(std::abs(e), max_volume_change) * sign_e;  // Update piston volume with saturation
  x_(2) = std::clamp(x_(2), 0.0, V_piston_max_);  // Ensure piston volume stays within limits
  last_piston_simulation_time_ = current_time;  // Update last update time
}

Matrix<double, Simulation::SIMU_NB_STATES, 1> Simulation::f() const
{
  Matrix<double, Simulation::SIMU_NB_STATES, 1> dxdt;
  dxdt(0) = x_(1);

  double weight = - robot_mass_ * g_physics_;
  double buoyancy = density_ * g_physics_ * compute_underwater_volume(x_(2));
  double damping = - (0.5 * density_ * std::abs(x_(1)) * drag_coeff_ * cross_section_) * x_(1);
  dxdt(1) = (weight + buoyancy + damping) / robot_mass_;
  dxdt(2) = 0.0;
  return dxdt;
}

double Simulation::compute_underwater_volume(double V_piston) const
{ 
  double V = V_body_ + 2 * V_thruster_ + 2 * V_arm_ + V_piston;
  double Va = 0.0;
  double z = x_(0);
  if (z <= 0) {
      Va = 0;
  } else if (z < a_/2-h_/2) {
      Va = V_body_ * z/a_;
  } else if (z < a_/2-f_/2) {
      Va = V_body_ * z/a_ + 2*V_thruster_ * (z - (a_/2 - h_/2))/h_;
  } else if (z < a_/2+f_/2) {
      Va = V_body_ * z/a_ + 2*V_thruster_ * (z - (a_/2 - h_/2))/h_  + 2*V_arm_ * (z - (a_/2 - f_/2))/f_;
  } else if (z < a_/2+h_/2) {
      Va = V_body_ * z/a_ + 2*V_thruster_ * (z - (a_/2 - h_/2))/h_  + 2*V_arm_;
  } else if (z < a_) {
      Va = V_body_ * z/a_ + 2*V_arm_ + 2*V_thruster_;
  } else {
      if (V_piston == 0) {
          Va = V_body_ + 2*V_arm_ + 2*V_thruster_;
      } else {
          Va = V_body_ + 2*V_arm_ + 2*V_thruster_ + V_piston * (z-a_)/(V_piston / (M_PI * pow(i_ / 2, 2)));
      }
  }
  double Vw = V - Va;
  return Vw;
}