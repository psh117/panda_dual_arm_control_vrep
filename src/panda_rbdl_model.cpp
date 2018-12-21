#include "panda_rbdl_model.h"

PandaRBDLModel::PandaRBDLModel()
{
  q_temp_.setZero(kNumJoints);
  qdot_temp_.setZero(kNumJoints);
  qddot_temp_.setZero(kNumJoints);
  j_temp_.setZero(6, kNumJoints);

  g_temp_.resize(kNumJoints);
  m_temp_.resize(kNumJoints, kNumJoints);

  model_.gravity = Vector3d(0., 0, -GRAVITY);
  double mass[kNumJoints];
  mass[0] = 1.0;
  mass[1] = 1.0;
  mass[2] = 1.0;
  mass[3] = 1.0;
  mass[4] = 1.0;
  mass[5] = 1.0;
  mass[6] = 1.0;

  Vector3d axis[kNumJoints];
  axis[0] = Eigen::Vector3d::UnitZ();
  axis[1] = Eigen::Vector3d::UnitY();
  axis[2] = Eigen::Vector3d::UnitZ();
  axis[3] = -1.0*Eigen::Vector3d::UnitY();
  axis[4] = Eigen::Vector3d::UnitZ();
  axis[5] = -1.0*Eigen::Vector3d::UnitY();
  axis[6] = -1.0*Eigen::Vector3d::UnitZ();


  Eigen::Vector3d global_joint_position[kNumJoints];

  global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
  global_joint_position[1] = global_joint_position[0];
  global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
  global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
  global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

  joint_posision_[0] = global_joint_position[0];
  for (int i = 1; i < kNumJoints; i++)
    joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

  com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
  com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
  com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
  com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
  com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
  com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
  com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

  for (int i = 0; i < kNumJoints; i++)
    com_position_[i] -= global_joint_position[i];

  RigidBodyDynamics::Math::Vector3d inertia[kNumJoints];
  for (int i = 0; i < kNumJoints; i++)
    inertia[i] = Eigen::Vector3d::Identity() * 0.001;

  for (int i = 0; i < kNumJoints; i++) {
    body_[i] = RigidBodyDynamics::Body(mass[i], com_position_[i], inertia[i]);
    joint_[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[i]);
    if (i == 0)
      body_id_[i] = model_.AddBody(0, RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    else
      body_id_[i] = model_.AddBody(body_id_[i - 1], RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
  }
}
void PandaRBDLModel::update(PandaRobotState &state){
  q_temp_ = state.q;
  qdot_temp_ = state.qdot;
  RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_temp_, &qdot_temp_, NULL);
  state.xp = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_temp_, body_id_[kNumJoints - 1], com_position_[kNumJoints - 1], true);

  Matrix3d xr = RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_temp_, body_id_[kNumJoints - 1], true).transpose();
  Matrix3d body_to_ee_rotation;
  body_to_ee_rotation.setIdentity();
  body_to_ee_rotation(1, 1) = -1;
  body_to_ee_rotation(2, 2) = -1;
  state.xr = xr * body_to_ee_rotation;

  RigidBodyDynamics::CalcPointJacobian6D(model_, q_temp_, body_id_[kNumJoints - 1], com_position_[kNumJoints - 1], j_temp_, true);


  RigidBodyDynamics::NonlinearEffects(model_, q_temp_, Vector7d::Zero(), g_temp_);
  RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_temp_, m_temp_, true);

  state.g = g_temp_;
  state.m = m_temp_;
  state.m_inverse = state.m.inverse();

  for (int i = 0; i<2; i++)
  {
    state.j.block<3, kNumJoints>(i * 3, 0) = j_temp_.block<3, kNumJoints>(3 - i * 3, 0);
  }
}
Affine3d PandaRBDLModel::getTransform(const Vector7d &q)
{
  Affine3d transform;
  transform.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q, body_id_[kNumJoints - 1], com_position_[kNumJoints - 1], false);

  transform.linear() = RigidBodyDynamics::CalcBodyWorldOrientation
      (model_, q, body_id_[kNumJoints - 1], true).transpose();

  Matrix3d body_to_ee_rotation;
  body_to_ee_rotation.setIdentity();
  body_to_ee_rotation(1, 1) = -1;
  body_to_ee_rotation(2, 2) = -1;

  transform.linear() = transform.linear() * body_to_ee_rotation;

  return transform;
}

Matrix<double,6,7> PandaRBDLModel::getJacobian(const Vector7d &q)
{
  Matrix<double,6,7> j;
  RigidBodyDynamics::CalcPointJacobian6D(model_, q, body_id_[kNumJoints - 1], com_position_[kNumJoints - 1], j_temp_, false);
  for (int i = 0; i<2; i++)
  {
    j.block<3, kNumJoints>(i * 3, 0) = j_temp_.block<3, kNumJoints>(3 - i * 3, 0);
  }
  return j;
}

