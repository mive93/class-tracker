#ifndef UKF_H
#define UKF_H

#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>
#include <iomanip>

#include "filter.h"

namespace tracking{

class UKF : public Filter
{
public:

  UKF();
  UKF(const int n_states, const float dt_, const FMatrixF &Q_, const FMatrixF &R_, const state &in_state);
  ~UKF();
  void step(const FMatrixF &H_, const Eigen::VectorXf &z);

private:
  state stateTransition(const state &x);
};

}

#endif /*UKF_H*/
