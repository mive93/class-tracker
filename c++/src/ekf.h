#ifndef EKF_H
#define EKF_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

#include <iomanip>

// #define COL_CYANB "\033[1;36m"
// #define COL_END "\033[0m"
// // Simple Timer
// #define TIMER_START    \
//   timespec start, end; \
//   clock_gettime(CLOCK_MONOTONIC, &start);

// #define TIMER_STOP_C(col)                                             \
//   clock_gettime(CLOCK_MONOTONIC, &end);                               \
//   double t_ns = ((double)(end.tv_sec - start.tv_sec) * 1.0e9 +        \
//                  (double)(end.tv_nsec - start.tv_nsec)) /             \
//                 1.0e6;                                                \
//   std::cout << col << "EKF Time:" << std::setw(16) << t_ns << " ms\n" \
//             << COL_END;

// #define TIMER_STOP TIMER_STOP_C(COL_CYANB)

struct State
{
  float x_;
  float y_;
  float yaw_;
  float vel_;
  float yaw_rate_;

  State();
  State(float x, float y, float yaw, float vel, float yaw_rate);
  void print();
};

class EKF
{
public:
  using EKFMatrixF = Eigen::Matrix<float, -1, -1>;

  EKF(const int n_states, const float dt, const EKFMatrixF &Q, const EKFMatrixF &R, const State &in_state);
  EKF();
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
