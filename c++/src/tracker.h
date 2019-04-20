#ifndef TRACKER_H
#define TRACKER_H

#include "ekf.h"
#include "trackutils.h"
#include "plot.h"
#include <cstdlib>     



  

struct Tracker
{

    std::vector<Data> traj_;
    std::vector<State> z_list_;
    std::vector<State> pred_list_;
    EKF ekf_;
    int age_;
    int r_;
    int g_;
    int b_;
    

    Tracker(Data first_point, int initial_age, float dt, int n_states);
};

EKF EFKinitialize(float dt, int n_states, Data first_point);
bool predicate(const std::vector<float> &a, const std::vector<float> &b);
std::vector<std::vector<float>> computeDistance(std::vector<Data> old_points, std::vector<Data> new_points);
void deleteOldTrajectories(std::vector<Tracker> &trackers, int age_threshold);
void addNewTrajectories(std::vector<Tracker> &trackers, std::vector<Data> frame, std::vector<bool> used, std::vector<std::vector<float>> knn_res, int initial_age, int n_states, float dt);
void Track(std::vector<Data> frame, float dt, int n_states, int initial_age, int age_threshold, std::vector<Tracker> &trackers);
void TrackOnGivenData(std::vector<Data> data, float dt, int n_states);

#endif /*TRACKER_H*/