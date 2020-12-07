#ifndef TRACKER_H
#define TRACKER_H

#include "ekf.h"
#include "obj.h"
#include <cstdlib>
#include <deque>

#define MAX_HISTORY 200 

namespace tracking {
class Tracker
{
public:
    std::deque<obj_m> traj;
    std::deque<state> zList;
    std::deque<state> predList;
    EKF ekf;
    int age;
    int r;
    int g;
    int b;
    int cl;
    int id;

    Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_);
    Tracker(const std::deque<obj_m>& traj,const std::deque<state>& zList, const std::deque<state>& predList,const EKF& ekf,const int age, const int r, const int g, const int b, const int cl, const int id);

private:
    Tracker();
    EKF ekfInitialize(const float dt, const int n_states, const obj_m &first_point);
};

}

#endif /*TRACKER_H*/