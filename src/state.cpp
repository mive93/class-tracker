#include "state.h"

namespace tracking{

void state::print(){
      std::cout << "x: " << x << "\ty: " << y << "\tyaw: " << yaw << "\tv: " << vel << "\tyaw rate: " << yawRate << std::endl;
}

Eigen::VectorXf StateIntoVector(const state &x, int n_states)
{
    Eigen::VectorXf v(n_states);
    v(0) = x.x;
    v(1) = x.y;
    v(2) = x.yaw;
    v(3) = x.vel;
    v(4) = x.yawRate;
    return v;
}

state VectorIntoState(const Eigen::VectorXf &v)
{
    state x(v(0), v(1), v(2), v(3), v(4));
    return x;
}

}