#include "Tracker.h"

Tracker::Tracker(const Data &first_point, const int initial_age, const float dt, const int n_states, const int id)
{
    traj_.push_back(first_point);
    age_ = initial_age;
    ekf_ = EFKinitialize(dt, n_states, first_point);

    r_ = rand() % 256;
    g_ = rand() % 256;
    b_ = rand() % 256;

    class_ = first_point.class_;
    id_ = id;
}

EKF Tracker::EFKinitialize(const float dt, const int n_states, const Data &first_point)
{
    EKF::EKFMatrixF Q = EKF::EKFMatrixF::Zero(n_states, n_states);
    EKF::EKFMatrixF R = EKF::EKFMatrixF::Zero(n_states, n_states);
    Q.diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);
    State s(first_point.x_, first_point.y_, 0, 0, 0);
    EKF ekf(n_states, dt, Q, R, s);
    //ekf.printInternalState();
    return ekf;
}
