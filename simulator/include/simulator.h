#pragma once
#include "safety_checks.h"
#include <matrix/math.hpp>
#include <yaml-cpp/yaml.h>

/// Represents the quadcopter
class Simulator {

private:
  // Euler integration timestep
  float dt_ = 0;
  // Number of numerical integration steps
  float euler_steps_ = 0;
  // Activate fastdds pose publisher
  bool pose_pub_flag_ = false;
  // SImulation timestep in milliseconds
  int sim_time_ = 0;

  matrix::Vector3f position_;
  matrix::Vector3f velocity_;
  matrix::Quatf orientation_;
  matrix::Vector3f angular_velocity_;
  /// Orientation of the rigid body as Quaternion (q_0, q_x, q_y, q_z)

public:
  // Loads the quadcopter properties from the yaml file
  void set_parameters(const std::string &path);

  // Integrators
public:
  void simulate_step(const matrix::Vector3f position,
                     const matrix::Vector3f velocity,
                     const matrix::Vector3f acceleration,
                     const matrix::Quatf orientation,
                     const matrix::Vector3f angular_velocity,
                     const matrix::Vector3f angular_acceleration);

  // Forward euler integrator
  matrix::Vector3f euler_forward_step(const matrix::Vector3f x_k,
                                      const matrix::Vector3f f_dot_x);

public:
  /// Getter function
  const float &dt() const { return dt_; }
  /// Getter function
  const float &euler_steps() const { return euler_steps_; }
  /// Getter function
  const int &sim_time() const { return sim_time_; }
  /// Getter function
  const bool &pose_pub_flag() const { return pose_pub_flag_; }
  /// Getter function
  const matrix::Vector3f &position() const { return position_; }
  /// Getter function
  const matrix::Vector3f &velocity() const { return velocity_; }
  /// Getter function
  const matrix::Quatf &orientation() const { return orientation_; }
  /// Getter function
  const matrix::Vector3f &angular_velocity() const { return angular_velocity_; }

public:
  /// Setter function
  void set_dt(float dt) { dt_ = dt; }
  /// Setter function
  void set_euler_steps(float euler_steps) { euler_steps_ = euler_steps; }
};
