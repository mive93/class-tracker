#ifndef TRACKING_H
#define TRACKING_H

#include "ekf.h"
#include "Data.h"
#include "plot.h"
#include "Tracker.h"
#include <cstdlib>

struct Knn_infos
{
    int obj_id_prev_   = -1;
    int obj_id_curr_   = -1;
    float dist_        =  0;  //distance
};

bool compareKnn_infos(const Knn_infos & a, const Knn_infos& b);

/* For each object in the new frame this method computes the distance from 
    * each object in the old frame and save the smallest one. 

    * Input
    * @param old_points: old frame objects
    * @param new_points: new frame objects
    *
    * @return a vector of float vectors containing 
    * <old_frame_obj_index, distance, new_frame_obj_index>, 
    * ordered for old_frame_obj_index first(ascending), distance second(ascending). 
    */
std::vector<Knn_infos> computeDistance(const std::vector<Data> &old_points, const std::vector<Data> &new_points);

#define MAX_INDEX 2048

class Tracking
{
    std::vector<Tracker> trackers_;

    bool * tracker_indexes_ = nullptr;
    int cur_index           = 0;
    int ekf_states_         = 5;
    int initial_age_        = 5;
    int age_threshold_      = 0;
    float dt_               = 0;

    void deleteOldTrajectories(bool verbose=false);
    void addNewTrajectories(const std::vector<Data> &frame, const std::vector<bool> &used, const std::vector<Knn_infos> &knn_res, bool verbose=false);
    void nearestNeighbor(const std::vector<Data> &frame, std::vector<Knn_infos>& knn_res, std::vector<bool>& used, std::vector<Data>& new_trajs);
    void kalmanStep(const std::vector<Data>& new_trajs);
    int getTrackerIndex();
    

public:
    Tracking(const int n_states = 5, const float dt = 0.03, const int initial_age = 5, const int age_threshold = 0);
    ~Tracking();
    void Track(const std::vector<Data> &frame, bool verbose=false);
    void TrackOnGivenData(const std::vector<Data> &data, bool verbose=false);
    std::vector<Tracker> getTrackers();

};

#endif /*TRACKING_H*/