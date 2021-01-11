#include "Tracker.h"

namespace tracking{
Tracker::Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_)
{
    traj.push_back(first_point);
    trajEkf.push_back(first_point);
    age = initial_age;
    ukf = ukfInitialize(dt, n_states, first_point);

    r = rand() % 256;
    g = rand() % 256;
    b = rand() % 256;

    cl = first_point.cl;
    id = id_;
}

Tracker::Tracker(const std::vector<obj_m>& traj,const std::vector<state>& zList, const std::vector<state>& predList,const UKF& ukf,const int age, const int r, const int g, const int b, const int cl, const int id)
{

    this->traj      = traj;
    this->trajEkf   = traj;
    this->zList     = zList;
    this->predList  = predList;
    this-> ukf      = ukf;
    this->age       = age;
    this->r         = r;
    this->g         = g;
    this->b         = b;
    this->cl        = cl;
    this->id        = id;
}

UKF Tracker::ukfInitialize(const float dt, const int n_states, const obj_m &first_point)
{
    UKF::UKFMatrixF Q = UKF::UKFMatrixF::Zero(n_states, n_states);
    UKF::UKFMatrixF R = UKF::UKFMatrixF::Zero(n_states, n_states);
    Q.diagonal() << 0.001*pow(3 * dt, 2), 0.001*pow(3 * dt, 2),0.001*pow(1 * dt, 2), 0.001*pow(25 * dt, 2), 0.001*pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.8, 2), pow(0.02, 2);
    state s(first_point.x, first_point.y, 0, 0, 0);
    UKF ukf(n_states, dt, Q, R, s);
    //ukf.printInternalState();
    return ukf;
}

}//namespace tracking