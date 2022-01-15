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

Tracker::Tracker(const std::deque<obj_m>& traj_,const std::deque<state>& zList_, const std::deque<state>& predList_,Filter* filter_,const int age_, const int r_, const int g_, const int b_, const int cl_, const int id_)
{
    this->traj          = traj_;
    this->trajFilter    = traj_;
    this->zList         = zList_;
    this->predList      = predList_;
    this-> filter       = filter_;
    this->age           = age_;
    this->r             = r_;
    this->g             = g_;
    this->b             = b_;
    this->cl            = cl_;
    this->id            = id_;
}

Filter* Tracker::filterInitialize(const float dt, const int n_states, const obj_m &first_point, Filters_t filter_type)
{
    tracking::FMatrixF Q = tracking::FMatrixF::Zero(n_states, n_states);
    tracking::FMatrixF R = tracking::FMatrixF::Zero(n_states, n_states);
    float dt2 = dt * dt;
    Q.diagonal() << 3.0f*3.0f*dt2, 3.0f*3.0f*dt2, 1.0f*1.0f*dt2, 25.0f*25.0f*dt2, 0.1f*0.1f*dt2;
    R.diagonal() << 0.5f*0.5f, 0.5f*0.5f, 0.1f*0.1f, 0.8f*0.8f, 0.02f*0.02f;
    state s(first_point.x, first_point.y, 0, 0, 0);

    if(filter_type == Filters_t::EKF_t){
        EKF * ekf = new EKF(n_states, dt, Q, R, s);
        return ekf;    
    }

    UKF * ukf = new UKF(n_states, dt, Q, R, s);
        return ukf;
}

}//namespace tracking