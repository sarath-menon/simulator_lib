#include "simulator.h"

void Simulator::set_parameters(std::string path) {

  // Load simulation properties from yaml file
  YAML::Node yaml_file = YAML::LoadFile(path);

  dt_ = yaml_file["dt"].as<float>();

  euler_steps_ = yaml_file["euler_steps"].as<float>();

  pose_pub_flag_ = yaml_file["pose_pub"].as<bool>();

  sim_time_ = dt_ * 1000;
}

void Simulator::simulate_step(const matrix::Vector3f position,
                              const matrix::Vector3f velocity,
                              const matrix::Vector3f acceleration,
                              const matrix::Quatf orientation,
                              const matrix::Vector3f angular_velocity,
                              const matrix::Vector3f angular_acceleration) {

  // Translation
  position_ = euler_forward_step(position, velocity);
  velocity_ = euler_forward_step(velocity, acceleration);

  // Rotation
  // Compute quaternion derivative
  const matrix::Quatf orientation_dot =
      matrix::Quatf::expq(0.5f * dt_ * angular_velocity);

  orientation_ = orientation * orientation_dot;
  orientation_.normalize();

  angular_velocity_ =
      euler_forward_step(angular_velocity, angular_acceleration);
}

matrix::Vector3f Simulator::euler_forward_step(const matrix::Vector3f x_k,
                                               const matrix::Vector3f f_dot_x) {
  return x_k + (f_dot_x * dt_);
}
