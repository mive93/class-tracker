#ifndef EKF_H
#define EKF_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <iostream>

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
  private:
    int n_states_;
    float dt_;
    State x_est_;
    Eigen::MatrixXf *Q_;
    Eigen::MatrixXf *R_;
    Eigen::MatrixXf *P_;
    Eigen::MatrixXf *H_;

    State StateTransition();
    Eigen::MatrixXf Jacobian(State x);

  public:
    EKF(int n_states, float dt, Eigen::MatrixXf *Q, Eigen::MatrixXf *R, State in_state);
    ~EKF();
    void printInternalState();
    void EKFStep(Eigen::MatrixXf H, Eigen::VectorXf z);
    State getEstimatedState();
    

};

#endif /*EKF_H*/