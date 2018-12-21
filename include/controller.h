#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <fstream>
#include "math_type_define.h"
#include "panda_rbdl_model.h"
#include "panda_robot_state.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

static constexpr int kDOF=14;
using MatrixQd=Eigen::Matrix<double,kDOF,kDOF>;
using VectorQd=Eigen::Matrix<double,kDOF,1>;


class ArmController
{
  enum class ArmIndex : int {kLeft=0, kRight=1};
  PandaRobotState arm_state_[2];

  PandaRBDLModel panda_model_;

  VectorQd q_desired_;
  VectorQd torque_desired_;

  unsigned long tick_;
  double play_time_;
  double hz_;
  double control_start_time_;

  std::string control_mode_;
  bool is_mode_changed_;

private:
  void printState();
  void moveJointPositionTorque(const VectorQd &target_position, double duration, ArmIndex arm_index);

public:
  void readData(const VectorQd &position, const VectorQd &velocity, const VectorQd &torque);
  void readData(const VectorQd &position, const VectorQd &velocity);
  const VectorQd & getDesiredPosition();
  const VectorQd & getDesiredTorque();

public:
  ArmController(double hz) :
    tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
  {
    initFile();
  }

  void setMode(const std::string & mode);
  void initFile();
  void initPosition();
  void compute();
private:
  constexpr static int NUM_HW_PLOT{3};
  ofstream debug_files[NUM_HW_PLOT];
  const string hw_plot_file_names_[NUM_HW_PLOT]
  {"q", "qd", "x"}; 	// 0 1 2

};
