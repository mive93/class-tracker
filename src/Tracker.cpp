#include "Tracker.h"

namespace tracking{
Tracker::Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_, Filters_t filter_type)
{
    traj.push_back(first_point);
    trajFilter.push_back(first_point);
    age = initial_age;
    filter =  filterInitialize(dt, n_states, first_point, filter_type);

    r = rand() % 256;
    g = rand() % 256;
    b = rand() % 256;

    cl = first_point.cl;
    id = id_;
}

Tracker::Tracker(const std::deque<obj_m>& traj,const std::deque<state>& zList, const std::deque<state>& predList,Filter* filter,const int age, const int r, const int g, const int b, const int cl, const int id)
{
    this->traj          = traj;
    this->trajFilter    = traj;
    this->zList         = zList;
    this->predList      = predList;
    this-> filter       = filter;
    this->age           = age;
    this->r             = r;
    this->g             = g;
    this->b             = b;
    this->cl            = cl;
    this->id            = id;
}

Filter* Tracker::filterInitialize(const float dt, const int n_states, const obj_m &first_point, Filters_t filter_type)
{
    tracking::FMatrixF Q = tracking::FMatrixF::Zero(n_states, n_states);
    tracking::FMatrixF R = tracking::FMatrixF::Zero(n_states, n_states);
    Q.diagonal() << 0.001*pow(3 * dt, 2), 0.001*pow(3 * dt, 2),0.001*pow(1 * dt, 2), 0.001*pow(25 * dt, 2), 0.001*pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.8, 2), pow(0.02, 2);
    state s(first_point.x, first_point.y, 0, 0, 0);

    if(filter_type == Filters_t::UKF_t){
        UKF * ukf = new UKF(n_states, dt, Q, R, s);
        return ukf;
    }
}

}//namespace tracking