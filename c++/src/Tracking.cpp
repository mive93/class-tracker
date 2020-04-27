#include "Tracking.h"

namespace tracking{

bool compareKnn_infos(const knn_infos & a, const knn_infos& b){
    // use to order for old_frame_obj_index first(ascending), distance second(ascending). 
    return (a.objIdPrev < b.objIdPrev || (a.objIdPrev == b.objIdPrev && a.dist < b.dist));
}

std::vector<knn_infos> computeDistance(const std::vector<obj_m> &old_points, const std::vector<obj_m> &new_points){

    /* for(auto o: old_points)
        o.print();

    for(auto n: new_points)
        n.print();     */
    
    float dist;
    std::vector<knn_infos> knn_res;
    for (size_t i = 0; i < new_points.size(); ++i){
        knn_infos infos;
        infos.objIdCurr = i;

        for (size_t j = 0; j < old_points.size(); ++j){
            dist = sqrt(pow(old_points[j].x - new_points[i].x, 2) + pow(old_points[j].y - new_points[i].y, 2));
            if (j == 0){
                infos.objIdPrev = j;
                infos.dist = dist;
            }
            else{
                if (dist < infos.dist && old_points[j].cl == new_points[i].cl){
                    infos.objIdPrev = j;
                    infos.dist = dist;
                }
            }
        }
        knn_res.push_back(infos);
    }

    std::sort(knn_res.begin(), knn_res.end(), compareKnn_infos);

    /* for(auto r: knn_res){
        for(auto i: r)
            std::cout<<i<<" ";
        std::cout<<std::endl;
    } */

    return knn_res;
}

Tracking::Tracking(const int n_states, const float dt_, const int initial_age) : ekfStates(n_states), dt(dt_), initialAge(initial_age){
    trackerIndexes = new bool[MAX_INDEX];
    for(int i=0; i<MAX_INDEX; ++i)
        trackerIndexes[i]= false;
}
Tracking::~Tracking() {
    // delete[] trackerIndexes; //FIXME
}

void Tracking::deleteOldTrajectories(bool verbose){
    if(trackers.size()){
        std::vector<Tracker> valid_trackers;

        for (size_t i = 0; i < trackers.size(); ++i){
            if (trackers[i].age > ageThreshold){
                valid_trackers.push_back(trackers[i]);
            }
            else{
                trackerIndexes[trackers[i].id] = false;
                if(verbose){
                    std::cout << "Deleting tracker " << trackers[i].id<< std::endl;
                    if (trackers[i].zList.size() > 10)
                        plotTruthvsPred(trackers[i].zList, trackers[i].predList);
                }
            }
        }
        trackers.swap(valid_trackers);
    }
}

void Tracking::addNewTrajectories(const std::vector<obj_m> &frame, const std::vector<bool> &used, const std::vector<knn_infos> &knn_res, bool verbose){
    for (size_t i = 0; i < knn_res.size(); ++i){
        if (!used[i]){
            trackers.push_back(Tracker(frame[knn_res[i].objIdCurr], initialAge, dt, ekfStates, this->getTrackerIndex()));
            if(verbose)
                std::cout << "Adding tracker " << trackers[trackers.size()-1].id<< std::endl;
        }
    }
}

void Tracking::nearestNeighbor(const std::vector<obj_m> &frame, std::vector<knn_infos>& knn_res, std::vector<bool>& used, std::vector<obj_m>& new_trajs){
    
    std::vector<obj_m> prev_trajs;
    for (auto t : trackers)
        prev_trajs.push_back(t.traj.back());

    int n_cur_trajs = prev_trajs.size();
    new_trajs.resize(n_cur_trajs);

    knn_res = computeDistance(prev_trajs, frame);
    used.resize(knn_res.size(), 0);
    std::vector<float> max_distance(n_cur_trajs, 15);

    for (size_t i = 0; i < trackers.size(); i++)
        trackers[i].age--;

    int prev_i, cur_i;
    float dist;
    for (size_t i = 0; i < knn_res.size(); i++){
        prev_i = knn_res[i].objIdPrev;
        dist = knn_res[i].dist;
        cur_i = knn_res[i].objIdCurr;

        if (n_cur_trajs > 0 && prev_i <= n_cur_trajs && dist < max_distance[prev_i]){
            trackers[prev_i].traj.push_back(frame[cur_i]);

            new_trajs[prev_i] = frame[cur_i];
            max_distance[prev_i] = dist;
            trackers[prev_i].age += 1;
            used[i] = 1;
        }
    }
}

void Tracking::kalmanStep(const std::vector<obj_m>& new_trajs){
    EKF::EKFMatrixF H = EKF::EKFMatrixF::Zero(ekfStates, ekfStates);
    Eigen::VectorXf z(ekfStates);

    for (size_t i = 0; i < new_trajs.size(); i++){
        z << new_trajs[i].x, new_trajs[i].y, 0, 0, 0;

        if (new_trajs[i].x == 0 && new_trajs[i].y == 0 && new_trajs[i].frame == -1){
            H(0, 0) = 0;
            H(1, 1) = 0;
        }
        else{
            H(0, 0) = 1;
            H(1, 1) = 1;
            trackers[i].zList.push_back(state(new_trajs[i].x, new_trajs[i].y, 0, 0, 0));
        }

        //trackers[i].ekf.printInternalState();
        trackers[i].ekf.ekfStep(H, z);

        trackers[i].predList.push_back(trackers[i].ekf.getEstimatedState());
    }
}

int Tracking::getTrackerIndex(){
    for(int i = curIndex; i < curIndex + MAX_INDEX; ++i ){
        int index = i % MAX_INDEX;
        if(!trackerIndexes[index]){
            trackerIndexes[index] = true;
            curIndex = index+1;
            return index;
        }
    }
    std::cerr << "\nProblem with tracker indexes, aborting...\n"; 
    exit(EXIT_FAILURE);  
}

void Tracking::track(const std::vector<obj_m> &frame, bool verbose)
{
    //delete trajectories not recently updated
    this->deleteOldTrajectories(verbose);

    //NN: associate new points with prevoius trajectories
    std::vector<knn_infos> knn_res;
    std::vector<bool> used;
    std::vector<obj_m> new_trajs;
    this->nearestNeighbor(frame, knn_res, used, new_trajs);

    //Kalman step
    this->kalmanStep(new_trajs);

    //Add new trajectories from points from the current frame that were not associated
    this->addNewTrajectories(frame, used, knn_res, verbose);
}

void Tracking::trackOnGivenData(const std::vector<obj_m> &data, bool verbose)
{
    int frame_id = 0;
    std::vector<obj_m> cur_frame;
    std::vector<Tracker> trackers;

    std::cout << "Start" << std::endl;

    for (auto d : data)
    {
        if (d.frame != frame_id){   
            track(cur_frame,verbose);
            cur_frame.clear();
            frame_id = d.frame;
        }
        cur_frame.push_back(d);
    }

    if(verbose){
        for (size_t i = 0; i < trackers.size(); i++)
            if (trackers[i].zList.size() > 10)
                plotTruthvsPred(trackers[i].zList, trackers[i].predList);
    }

    std::cout << "End." << std::endl;
}

std::vector<Tracker> Tracking::getTrackers(){
    return trackers;
}

void Tracking::setAgeThreshold(const int age_threshold){
    ageThreshold = age_threshold;
}

}//namespace tracking