#ifndef UKF_H
#define UKF_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <iostream>

#include <iomanip>

namespace tracking{

struct state
{
  float x          = 0;
  float y          = 0;
  float yaw        = 0;
  float vel        = 0;
  float yawRate   = 0;

  state() {}
  state(const float x_, const float y_, const float yaw_, const float vel_, 
        const float yaw_rate): x(x_), y(y_), yaw(yaw_), vel(vel_), 
        yawRate(yaw_rate) {}
  void print();
};

class UKF
{
public:
  using UKFMatrixF = Eigen::Matrix<float, -1, -1>;

  UKF();
  UKF(const int n_states, const float dt_, const UKFMatrixF &Q_, const UKFMatrixF &R_, const state &in_state);
  ~UKF();
  void printInternalState();
  void ukfStep(const UKFMatrixF &H_, const Eigen::VectorXf &z);
  state getEstimatedState();

private:
  int nStates;
  float dt;
  state xEst;
  UKFMatrixF Q;
  UKFMatrixF R;
  UKFMatrixF P;
  UKFMatrixF H;

  state stateTransition(const state &x);
};

}

#endif /*UKF_H*/
