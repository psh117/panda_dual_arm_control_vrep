#pragma once

#include <Eigen/Dense>

struct PandaRobotState
{
  static constexpr int kArmDOF=7;
  using Vector7d=Eigen::Matrix<double,kArmDOF,1>;
  using Matrix7d=Eigen::Matrix<double,kArmDOF,kArmDOF>;
  using Matrix6d=Eigen::Matrix<double,6,6>;

  // Joint space -----
  // Initial states
  Vector7d q_0;
  Vector7d qdot_0;

  // Current states
  Vector7d q;
  Vector7d qdot;
  Vector7d qddot;
  Vector7d torque;

  // Desired states
  Vector7d q_desired;
  Vector7d torque_desired;

  // Task space -----
  // Initial states
  Eigen::Vector3d xp_0;
  Eigen::Matrix3d xr_0;

  // Current states
  Eigen::Vector3d xp;
  Eigen::Matrix3d xr;

  // Desired states
  Eigen::Vector3d xp_desired;
  Eigen::Matrix3d xr_desired;

  // Dynamcis (without coupling)
  Vector7d g; // Gravity torque
  Matrix7d m; // Mass matrix
  Matrix7d m_inverse; // Inverse of mass matrix
  Matrix6d lambda;
  Matrix6d lambda_inverse;

  // Jacobian to EE
  Eigen::Matrix<double, 3, 7> j_v;	// Linear velocity Jacobian matrix
  Eigen::Matrix<double, 3, 7> j_w;	// Angular veolicty Jacobain matrix
  Eigen::Matrix<double, 6, 7> j;	// Full basic Jacobian matrix
  Eigen::Matrix<double, 7, 6> j_inverse;	// Jacobain inverse storage
};
