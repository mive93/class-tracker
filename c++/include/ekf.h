#ifndef EKF_H
#define EKF_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

#include <iomanip>

struct State
{
  float x_          = 0;
  float y_          = 0;
  float yaw_        = 0;
  float vel_        = 0;
  float yaw_rate_   = 0;

  State() {}
  State(const float x, const float y, const float yaw, const float vel, 
        const float yaw_rate): x_(x), y_(y), yaw_(yaw), vel_(vel), 
        yaw_rate_(yaw_rate) {}
  void print();
};

class EKF
{
public:
  using EKFMatrixF = Eigen::Matrix<float, -1, -1>;

  EKF();
  EKF(const int n_states, const float dt, const EKFMatrixF &Q, const EKFMatrixF &R, const State &in_state);
  ~EKF();
  void printInternalState();
  void EKFStep(const EKFMatrixF &H, const Eigen::VectorXf &z);
  State getEstimatedState();

private:
  int n_states_;
  float dt_;
  State x_est_;
  EKFMatrixF Q_;
  EKFMatrixF R_;
  EKFMatrixF P_;
  EKFMatrixF H_;

  State StateTransition();
  EKFMatrixF Jacobian(const State &x);
};

#endif /*EKF_H*/
