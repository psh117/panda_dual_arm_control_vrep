
#include <rbdl/rbdl.h>
#include "math_type_define.h"
#include "panda_robot_state.h"

using namespace Eigen;

class PandaRBDLModel
{
  static constexpr int kNumJoints=7;
  VectorXd q_temp_;	// For RBDL
  VectorXd qdot_temp_;
  VectorXd qddot_temp_;
  MatrixXd j_temp_;	// For RBDL
  MatrixXd m_temp_;
  VectorXd g_temp_;   // For RBD

  // for robot model construction
  RigidBodyDynamics::Model model_;
  unsigned int body_id_[kNumJoints];
  RigidBodyDynamics::Math::Vector3d com_position_[kNumJoints];
  Vector3d joint_posision_[kNumJoints];
  RigidBodyDynamics::Body body_[kNumJoints];
  RigidBodyDynamics::Joint joint_[kNumJoints];

public:
  PandaRBDLModel();
  void update(PandaRobotState &state);

  Affine3d getTransform(const Vector7d &q);
  Matrix<double,6,7> getJacobian(const Vector7d &q);
};
