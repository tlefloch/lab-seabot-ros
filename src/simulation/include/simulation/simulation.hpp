#ifndef SIMULATION_SIMULATION_HPP_
#define SIMULATION_SIMULATION_HPP_

#include <Eigen/Dense>
#include <random>

#include "rclcpp/rclcpp.hpp"

using Eigen::Matrix;

class Simulation
{
public:
  Simulation();
  void init(rclcpp::Time current_time);
  void simulate_pressure_sensor();
  void simulate_physics(std::chrono::milliseconds dt);
  void simulate_piston(rclcpp::Time current_time);
  void update_piston_setpoint(double piston_volume);

  // Robot dimensions and properties
  double robot_size_ = 1.1;
  double a_ = robot_size_ * 0.7;
  double b_ = robot_size_ * 0.1;
  double c_ = robot_size_ * 0.2;
  double d_ = robot_size_ * 0.1;
  double e_ = d_;
  double f_ = d_ * 0.3;
  double g_ = e_ * 0.8;
  double h_ = f_ * 1.5;
  double i_ = d_ * 0.5;

  double V_body_ = a_ * M_PI * pow(d_ / 2, 2);
  double V_thruster_ = h_ * M_PI * pow(g_ / 2, 2);
  double V_arm_ = e_ * M_PI * pow(f_ / 2, 2);

  double robot_mass_ = 8.58;  // in kg
  double V_piston_max_ = c_ * M_PI * pow(i_ / 2, 2);  // Maximum piston volume in cubic meters
  double drag_coeff_ = 0.8;  // Drag coefficient
  double cross_section_ = 0.5;  // Reference area for drag force in m

  double pressure_m_ = 0.;   /// Simulated (noisy) value of the pressure sensor in bar

  // State
  static const int SIMU_NB_STATES = 3;  // Number of states in the simulation
  /**
   * @brief Simulation state vector, containing:
   * z: Depth (meters)
   * vz: Vertical velocity (meters/second)
   * Vp: Piston volume out of the body (cubic meters)
   * 
   */
  Matrix<double, SIMU_NB_STATES, 1> x_ = Matrix < double, SIMU_NB_STATES, 1 > ::Zero();  // State vector
  double abs_pressure_ = 0.0;  // Absolute pressure in bar

private:
  
  // Physics
  double g_physics_ = 9.81;  // Gravitational acceleration
  double salinity_ = 35.0;  // Salinity in PSU
  double temperature_ = 15.0;  // Temperature in degrees Celsius
  double density_ = 1025.0;  // Density of seawater in kg/m^3
  double zero_pressure_ = 1.0;  // Pressure at the surface in bar

  double compute_underwater_volume(double V_piston) const;  // Compute the volume of the robot that is underwater based on the current state

  // Pressure sensor
  const double pressure_sensor_mean_ = 0.0;
  double pressure_sensor_stddev_ = 0.002;   // in bar (2mbar)
  std::default_random_engine generator_;
  std::normal_distribution<double> pressure_sensor_dist_{pressure_sensor_mean_, pressure_sensor_stddev_};

  // Piston
  double piston_setpoint_ = V_piston_max_;  // Desired piston volume in cubic meters
  rclcpp::Time last_piston_simulation_time_;
  double piston_max_flow_rate_ = 2e-4;  // Maximum piston flow rate in cubic meters per second

  Matrix<double, SIMU_NB_STATES, 1> f() const;

};

#endif  // SIMULATION_SIMULATION_HPP_