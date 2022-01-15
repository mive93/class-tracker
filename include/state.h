#ifndef STATE_H
#define STATE_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

namespace tracking{

struct state
{
  float x          = 0.0f;
  float y          = 0.0f;
  float yaw        = 0.0f;
  float vel        = 0.0f;
  float yawRate    = 0.0f;

  state() {}
  state(const float x_, const float y_, const float yaw_, const float vel_, 
        const float yaw_rate): x(x_), y(y_), yaw(yaw_), vel(vel_), 
        yawRate(yaw_rate) {}
  void print();
};

Eigen::VectorXf StateIntoVector(const state &x, int n_states);
state VectorIntoState(const Eigen::VectorXf &v);

}

#endif /*STATE_H*/
