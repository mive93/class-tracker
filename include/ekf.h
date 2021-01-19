#ifndef EKF_H
#define EKF_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

#include <iomanip>

#include "state.h"
#include "filter.h"

namespace tracking{
class EKF : public Filter
{
public:

  EKF();
  EKF(const int n_states, const float dt_, const FMatrixF &Q_, const FMatrixF &R_, const state &in_state);
  ~EKF();
  void step(const FMatrixF &H_, const Eigen::VectorXf &z);

private:

  state stateTransition();
  FMatrixF jacobian(const state &x);
};

}

#endif /*EKF_H*/
