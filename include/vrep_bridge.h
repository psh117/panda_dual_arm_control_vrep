#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

using namespace std;

extern "C" {
#include "extApi.h"
}

#ifndef DO_NOT_USE_SHARED_MEMORY
#ifndef SHARED_MEMORY_PORT
#define SHARED_MEMORY_PORT -3
#endif
#endif

class VRepBridge
{
public:
  // TODO: Modify the value of kDOF to fit your application

  static constexpr int kDOF=7+7; // Left, Right, Mobiles

	enum ControlMode { CTRL_POSITION, CTRL_VELOCITY, CTRL_TORQUE };

	VRepBridge(ControlMode mode = CTRL_POSITION);
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

  void setDesiredPosition(const Eigen::Matrix<double, kDOF, 1> & desired_q);
  void setDesiredTorque(const Eigen::Matrix<double, kDOF, 1> & desired_torque);
  const Eigen::Matrix<double, kDOF, 1> & getPosition();
  const Eigen::Matrix<double, kDOF, 1> & getVelocity();

	const size_t getTick() { return tick_; }

private:
  Eigen::Matrix<double, kDOF, 1> current_q_;
  Eigen::Matrix<double, kDOF, 1> current_q_dot_;
  Eigen::Matrix<double, kDOF, 1> desired_q_;
  Eigen::Matrix<double, kDOF, 1> desired_torque_;

	simxInt clientID_;
  simxInt motorHandle_[kDOF];	/// < Depends on simulation envrionment
	simxInt objectHandle_;

	size_t tick_{ 0 };
	
	ControlMode control_mode_;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
