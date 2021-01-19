#ifndef TRACKER_H
#define TRACKER_H

#include "ukf.h"
#include "obj.h"
#include <cstdlib>


namespace tracking {
class Tracker
{
public:
    std::vector<obj_m> traj;
    std::vector<obj_m> trajFilter;
    std::vector<state> zList;
    std::vector<state> predList;
    Filter *filter;
    int age;
    int r;
    int g;
    int b;
    int cl;
    int id;

    Tracker(const obj_m &first_point, const int initial_age, const float dt, const int n_states, const int id_, Filters_t filter_type);
    Tracker(const std::vector<obj_m>& traj,const std::vector<state>& zList, const std::vector<state>& predList,Filter* filter,const int age, const int r, const int g, const int b, const int cl, const int id);

private:
    Tracker();
    Filter* filterInitialize(const float dt, const int n_states, const obj_m &first_point, Filters_t filter_type);
};

}

#endif /*TRACKER_H*/