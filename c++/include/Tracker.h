#ifndef TRACKER_H
#define TRACKER_H

#include "ekf.h"
#include "Data.h"
#include <cstdlib>

class Tracker
{
public:
    std::vector<Data> traj_;
    std::vector<State> z_list_;
    std::vector<State> pred_list_;
    EKF ekf_;
    int age_;
    int r_;
    int g_;
    int b_;
    int class_;
    int id_;

    Tracker(const Data &first_point, const int initial_age, const float dt, const int n_states, const int id);

private:
    Tracker();
    EKF EFKinitialize(const float dt, const int n_states, const Data &first_point);

};

#endif /*TRACKER_H*/