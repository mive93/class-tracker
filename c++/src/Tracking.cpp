#include "Tracking.h"

bool compareKnn_infos(const Knn_infos & a, const Knn_infos& b)
{
    // use to order for old_frame_obj_index first(ascending), distance second(ascending). 
    return (a.obj_id_prev_ < b.obj_id_prev_ || (a.obj_id_prev_ == b.obj_id_prev_ && a.dist_ < b.dist_));
}

std::vector<Knn_infos> computeDistance(const std::vector<Data> &old_points, const std::vector<Data> &new_points)
{

    /* for(auto o: old_points)
        o.print();

    for(auto n: new_points)
        n.print();     */
    
    float dist;
    std::vector<Knn_infos> knn_res;
    for (size_t i = 0; i < new_points.size(); ++i)
    {
        Knn_infos infos;
        infos.obj_id_curr_ = i;

        for (size_t j = 0; j < old_points.size(); ++j)
        {
            dist = sqrt(pow(old_points[j].x_ - new_points[i].x_, 2) + pow(old_points[j].y_ - new_points[i].y_, 2));
            if (j == 0)
            {
                infos.obj_id_prev_ = j;
                infos.dist_ = dist;
            }
            else
            {
                if (dist < infos.dist_ && old_points[j].class_ == new_points[i].class_)
                {
                    infos.obj_id_prev_ = j;
                    infos.dist_ = dist;
                }
            }
        }
        knn_res.push_back(infos);
    }

    std::sort(knn_res.begin(), knn_res.end(), compareKnn_infos);

    /* for(auto r: knn_res)
    {
        for(auto i: r)
            std::cout<<i<<" ";
        std::cout<<std::endl;
    } */

    return knn_res;
}

Tracking::Tracking(const int n_states, const float dt, const int initial_age, const int age_threshold) : ekf_states_(n_states), dt_(dt), initial_age_(initial_age), age_threshold_(age_threshold)
{
    tracker_indexes_ = new bool[MAX_INDEX];
    for(int i=0; i<MAX_INDEX; ++i)
        tracker_indexes_[i]= false;
}
Tracking::~Tracking() 
{
    // delete[] tracker_indexes_; //FIXME
}

void Tracking::deleteOldTrajectories(bool verbose)
{
    if(trackers_.size())
    {
        std::vector<Tracker> valid_trackers;

        for (size_t i = 0; i < trackers_.size(); ++i)
        {
            if (trackers_[i].age_ > age_threshold_)
            {
                valid_trackers.push_back(trackers_[i]);
            }
            else
            {
                tracker_indexes_[trackers_[i].id_] = false;
                if(verbose)
                {
                    std::cout << "Deleting tracker " << trackers_[i].id_<< std::endl;
                    if (trackers_[i].z_list_.size() > 10)
                        plotTruthvsPred(trackers_[i].z_list_, trackers_[i].pred_list_);
                }
            }
        }

        trackers_.swap(valid_trackers);
    }
}

void Tracking::addNewTrajectories(const std::vector<Data> &frame, const std::vector<bool> &used, const std::vector<Knn_infos> &knn_res, bool verbose)
{
    for (size_t i = 0; i < knn_res.size(); ++i)
    {
        if (!used[i])
        {
            trackers_.push_back(Tracker(frame[knn_res[i].obj_id_curr_], initial_age_, dt_, ekf_states_, this->getTrackerIndex()));
            if(verbose)
                std::cout << "Adding tracker " << trackers_[trackers_.size()-1].id_<< std::endl;
        }
    }
}

void Tracking::nearestNeighbor(const std::vector<Data> &frame, std::vector<Knn_infos>& knn_res, std::vector<bool>& used, std::vector<Data>& new_trajs)
{
    
    std::vector<Data> prev_trajs;
    for (auto t : trackers_)
        prev_trajs.push_back(t.traj_.back());

    int n_cur_trajs = prev_trajs.size();
    new_trajs.resize(n_cur_trajs);

    knn_res = computeDistance(prev_trajs, frame);
    used.resize(knn_res.size(), 0);
    std::vector<float> max_distance(n_cur_trajs, 15);

    for (size_t i = 0; i < trackers_.size(); i++)
        trackers_[i].age_--;

    int prev_i, cur_i;
    float dist;
    for (size_t i = 0; i < knn_res.size(); i++)
    {
        prev_i = knn_res[i].obj_id_prev_;
        dist = knn_res[i].dist_;
        cur_i = knn_res[i].obj_id_curr_;

        if (n_cur_trajs > 0 && prev_i <= n_cur_trajs && dist < max_distance[prev_i])
        {
            trackers_[prev_i].traj_.push_back(frame[cur_i]);

            new_trajs[prev_i] = frame[cur_i];
            max_distance[prev_i] = dist;
            trackers_[prev_i].age_ += 1;
            used[i] = 1;
        }
    }
}

void Tracking::kalmanStep(const std::vector<Data>& new_trajs)
{
    EKF::EKFMatrixF H = EKF::EKFMatrixF::Zero(ekf_states_, ekf_states_);
    Eigen::VectorXf z(ekf_states_);

    for (size_t i = 0; i < new_trajs.size(); i++)
    {

        z << new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0;

        if (new_trajs[i].x_ == 0 && new_trajs[i].y_ == 0 && new_trajs[i].frame_ == -1)
        {
            H(0, 0) = 0;
            H(1, 1) = 0;
        }
        else
        {
            H(0, 0) = 1;
            H(1, 1) = 1;
            trackers_[i].z_list_.push_back(State(new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0));
        }

        //trackers_[i].ekf_.printInternalState();
        trackers_[i].ekf_.EKFStep(H, z);

        trackers_[i].pred_list_.push_back(trackers_[i].ekf_.getEstimatedState());
    }
}

int Tracking::getTrackerIndex()
{
    for(int i = cur_index; i < cur_index + MAX_INDEX; ++i )
    {
        int index = i % MAX_INDEX;
        if(!tracker_indexes_[index])
        {
            tracker_indexes_[index] = true;
            cur_index = index+1;
            return index;
        }
    }
    std::cerr << "\nProblem with tracker indexes, aborting...\n"; 
    exit(EXIT_FAILURE);  
}

void Tracking::Track(const std::vector<Data> &frame, bool verbose)
{
    //delete trajectories not recently updated
    this->deleteOldTrajectories(verbose);

    //NN: associate new points with prevoius trajectories
    std::vector<Knn_infos> knn_res;
    std::vector<bool> used;
    std::vector<Data> new_trajs;
    this->nearestNeighbor(frame, knn_res, used, new_trajs);

    //Kalman step
    this->kalmanStep(new_trajs);

    //Add new trajectories from points from the current frame that were not associated
    this->addNewTrajectories(frame, used, knn_res, verbose);
}

void Tracking::TrackOnGivenData(const std::vector<Data> &data, bool verbose)
{
    int frame_id = 0;
    std::vector<Data> cur_frame;

    std::vector<Tracker> trackers_;

    std::cout << "Start" << std::endl;

    for (auto d : data)
    {
        if (d.frame_ != frame_id)
        {   
            Track(cur_frame,verbose);
            cur_frame.clear();
            frame_id = d.frame_;
        }
        cur_frame.push_back(d);
    }

    if(verbose)
    {
        for (size_t i = 0; i < trackers_.size(); i++)
            if (trackers_[i].z_list_.size() > 10)
                plotTruthvsPred(trackers_[i].z_list_, trackers_[i].pred_list_);
    }

    std::cout << "End." << std::endl;
}

std::vector<Tracker> Tracking::getTrackers()
{
    return trackers_;
}