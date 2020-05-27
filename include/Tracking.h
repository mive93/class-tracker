#ifndef TRACKING_H
#define TRACKING_H

#include "ekf.h"
#include "obj.h"
#include "plot.h"
#include "Tracker.h"
#include <cstdlib>

namespace tracking{

struct knn_infos
{
    int objIdPrev   = -1;
    int objIdCurr   = -1;
    float dist      =  0;  //distance
};

bool compareKnn_infos(const knn_infos & a, const knn_infos& b);

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
std::vector<knn_infos> computeDistance(const std::vector<obj_m> &old_points, const std::vector<obj_m> &new_points);

#define MAX_INDEX 32767

class Tracking
{
    bool * trackerIndexes   = nullptr;
    int curIndex            = 0;
    int ekfStates           = 5;
    int initialAge          = 5;
    int ageThreshold        = 0;
    float dt                = 0;

    void deleteOldTrajectories(bool verbose=false);
    void addNewTrajectories(const std::vector<obj_m> &frame, const std::vector<bool> &used, const std::vector<knn_infos> &knn_res, bool verbose=false);
    void nearestNeighbor(const std::vector<obj_m> &frame, std::vector<knn_infos>& knn_res, std::vector<bool>& used, std::vector<obj_m>& new_trajs);
    void kalmanStep(const std::vector<obj_m>& new_trajs);
    int getTrackerIndex();
    

public:
    std::vector<Tracker> trackers;

    Tracking(const int n_states = 5, const float dt_ = 0.03, const int initial_age = 5);
    ~Tracking();
    void track(const std::vector<obj_m> &frame, bool verbose=false);
    void trackOnGivenData(const std::vector<obj_m> &data, bool verbose=false);
    std::vector<Tracker> getTrackers();
    void setAgeThreshold(const int age_threshold);

};

}

#endif /*TRACKING_H*/