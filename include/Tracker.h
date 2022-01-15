#ifndef TRACKER_H
#define TRACKER_H

#include "ukf.h"
#include "ekf.h"
#include "obj.h"
#include <cstdlib>
#include <deque>

#define MAX_HISTORY 50 

namespace tracking {
class Tracker
{
public:
    std::deque<obj_m> traj;
    std::deque<obj_m> trajFilter;
    std::deque<state> zList;
    std::deque<state> predList;
    Filter *filter = nullptr;
    int age;
    int r;
    int g;
    int b;
    int cl;
    int id;

    Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_, Filters_t filter_type);
    Tracker(const std::deque<obj_m>& traj_,const std::deque<state>& zList_, const std::deque<state>& predList_,Filter* filter_,const int age_, const int r_, const int g_, const int b_, const int cl_, const int id_);

private:
    Tracker();
    Filter* filterInitialize(const float dt, const int n_states, const obj_m &first_point, Filters_t filter_type);
};

}

#endif /*TRACKER_H*/