#include "Tracker.h"

namespace tracking{
Tracker::Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_)
{
    traj.push_back(first_point);
    age = initial_age;
    ekf = ekfInitialize(dt, n_states, first_point);

    r = rand() % 256;
    g = rand() % 256;
    b = rand() % 256;

    cl = first_point.cl;
    id = id_;
}

EKF Tracker::ekfInitialize(const float dt, const int n_states, const obj_m &first_point)
{
    EKF::EKFMatrixF Q = EKF::EKFMatrixF::Zero(n_states, n_states);
    EKF::EKFMatrixF R = EKF::EKFMatrixF::Zero(n_states, n_states);
    Q.diagonal() << pow(3 * dt, 2), pow(3 * dt, 2), pow(1 * dt, 2), pow(25 * dt, 2), pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.8, 2), pow(0.02, 2);
    state s(first_point.x, first_point.y, 0, 0, 0);
    EKF ekf(n_states, dt, Q, R, s);
    //ekf.printInternalState();
    return ekf;
}

}//namespace tracking