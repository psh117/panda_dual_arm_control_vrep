#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>



void ArmController::compute()
{
  for(int i=0; i<2;i++)
  {
    panda_model_.update(arm_state_[i]);
  }

  if (is_mode_changed_)
  {
    is_mode_changed_ = false;

    control_start_time_ = play_time_;

    for(int i=0; i<2; i++)
    {
      arm_state_[i].q_0 = arm_state_[i].q;
      arm_state_[i].qdot_0 = arm_state_[i].qdot;

      arm_state_[i].xp_0 = arm_state_[i].xp;
      arm_state_[i].xr_0 = arm_state_[i].xr;
    }
  }

  if (control_mode_ == "joint_ctrl_init")
  {
    VectorQd target_position;
    target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
  }
  else
  {
    for(int i=0; i<2; i++)
    {
      torque_desired_.segment<7>(i*7) = arm_state_[i].g;
    }
  }

  printState();

  tick_++;
  play_time_ = tick_ / hz_;	// second
}

void ArmController::printState()
{
  // TODO: Modify this method to debug your code

  static int DBG_CNT = 0;
  if (DBG_CNT++ > hz_ / 50.)
  {
    DBG_CNT = 0;

    //		cout << "q now    :\t";
    //		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
    //		cout << "q desired:\t";
    //		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
    //		cout << "t desired:\t";
    //		cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
    //		cout << "x        :\t";
    //		cout << x_.transpose() << endl;
    //		cout << "R        :\t" << endl;
    //		cout << std::fixed << std::setprecision(3) << rotation_ << endl;

    //		Matrix<double, 6, 7>j;
    //    j <<  j_.block <3, kDOF>(0, 0),
    //      j_2_.block<3, kDOF>(0, 0);

    //		cout << "hw 3-1 jacobian:" << endl;
    //		cout << j << endl;

    //		Vector6d x;
    //		x << x_, x_2_;

    //		cout << "hw 3-1 x:\t" << x.transpose() << endl;
  }
}


void ArmController::moveJointPositionTorque(const VectorQd &target_position, double duration, ArmIndex arm_index)
{
  Matrix7d kp, kv;
  Vector7d q_cubic, qd_cubic;

  kp = Matrix7d::Identity() * 500.0;
  kv = Matrix7d::Identity() * 20.0;
  PandaRobotState &st = arm_state_[(int)arm_index];

  for (int i = 0; i < 7; i++)
  {
    qd_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
                                      control_start_time_ + duration, st.q_0(i), target_position(i), 0, 0);
    q_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
                                  control_start_time_ + duration, st.q_0(i), target_position(i), 0, 0);
  }

  torque_desired_.segment<7>((int)arm_index * 7) = st.m * (kp*(q_cubic - st.q) + kv*(qd_cubic - st.qdot)) + st.g;
}

// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initFile()
{
  for (int i = 0; i < NUM_HW_PLOT; i++)
  {
    debug_files[i].open(hw_plot_file_names_[i] + ".txt");
  }
}

void ArmController::readData(const VectorQd &position, const VectorQd &velocity, const VectorQd &torque)
{
  for (int i=0; i<2; i++)
  {
    arm_state_[i].q = position.segment<7>(i*7);
    arm_state_[i].qdot = velocity.segment<7>(i*7);
    arm_state_[i].torque = torque.segment<7>(i*7);
  }
}
void ArmController::readData(const VectorQd &position, const VectorQd &velocity)
{
  readData(position, velocity, VectorQd::Zero());
}

const VectorQd & ArmController::getDesiredPosition()
{
  return q_desired_;
}

const VectorQd & ArmController::getDesiredTorque()
{
  return torque_desired_;
}



void ArmController::initPosition()
{
  for(int i=0; i<2; i++)
  {
    arm_state_[i].q_desired = arm_state_[i].q_0 = arm_state_[i].q;
  }
}

// ----------------------------------------------------

