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
    int class_;

    Tracker(const Data &first_point, int initial_age, float dt, int n_states);
};

EKF EFKinitialize(float dt, int n_states, const Data &first_point);
bool predicate(const std::vector<float> &a, const std::vector<float> &b);
std::vector<std::vector<float>> computeDistance(const std::vector<Data> &old_points, const std::vector<Data> &new_points);
void deleteOldTrajectories(std::vector<Tracker> &trackers, int age_threshold, bool verbose=false);
void addNewTrajectories(std::vector<Tracker> &trackers, const std::vector<Data> &frame, const std::vector<bool> &used, const std::vector<std::vector<float>> &knn_res, int initial_age, int n_states, float dt);
void Track(const std::vector<Data> &frame, float dt, int n_states, int initial_age, int age_threshold, std::vector<Tracker> &trackers, bool verbose=false);
void TrackOnGivenData(const std::vector<Data> &data, float dt, int n_states, float verbose=false);

#endif /*TRACKER_H*/