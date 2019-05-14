#include "tracker.h"

Tracker::Tracker(const Data& first_point, int initial_age, float dt, int n_states)
{
    traj_.push_back(first_point);
    age_ = initial_age;
    ekf_ = EFKinitialize(dt, n_states, first_point);

    r_ = rand() % 256;
    g_ = rand() % 256;
    b_ = rand() % 256;

    class_ = first_point.class_;
}

EKF EFKinitialize(float dt, int n_states, const Data& first_point)
{
    Eigen::MatrixXf *Q = new Eigen::MatrixXf(n_states, n_states);
    Eigen::MatrixXf *R = new Eigen::MatrixXf(n_states, n_states);
    (*Q).setZero();
    (*R).setZero();
    (*Q).diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
    (*R).diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);
    State s(first_point.x_, first_point.y_, 0, 0, 0);
    EKF ekf(n_states, dt, Q, R, s);
    //ekf.printInternalState();
    return ekf;
}

bool predicate(const std::vector<float>& a, const std::vector<float>& b)
{
    return (a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]));
}

std::vector<std::vector<float>> computeDistance(const std::vector<Data>& old_points, const std::vector<Data>& new_points)
{

    /* for(auto o: old_points)
        o.print();

    for(auto n: new_points)
        n.print();     */

    int min_j;
    float min_d = 0;
    float dist;
    std::vector<float> infos;
    std::vector<std::vector<float>> knn_res;
    for (size_t i = 0; i < new_points.size(); i++)
    {
        for (size_t j = 0; j < old_points.size(); j++)
        {
            dist = sqrt(pow(old_points[j].x_ - new_points[i].x_, 2) + pow(old_points[j].y_ - new_points[i].y_, 2));
            if (j == 0)
            {
                min_j = j;
                min_d = dist;
            }
            else
            {
                if (dist < min_d)
                {
                    min_d = dist;
                    min_j = j;
                }
            }
        }
        infos.clear();
        infos.push_back(min_j);
        infos.push_back(min_d);
        infos.push_back(i);
        knn_res.push_back(infos);
    }

    std::sort(knn_res.begin(), knn_res.end(), predicate);

    /* for(auto r: knn_res)
    {
        for(auto i: r)
            std::cout<<i<<" ";
        std::cout<<std::endl;
    } */

    return knn_res;
}

void deleteOldTrajectories(std::vector<Tracker> &trackers, int age_threshold)
{
    std::vector<Tracker> valid_trackers;

    for (size_t i = 0; i < trackers.size(); i++)
    {
        if (trackers[i].age_ > age_threshold)
        {
            valid_trackers.push_back(trackers[i]);
        }
        else
        {
            std::cout << "Deleting a trajectory" << std::endl;
            /* if (trackers[i].z_list_.size() > 10)
                plotTruthvsPred(trackers[i].z_list_, trackers[i].pred_list_); */
        }
    }

    trackers.swap(valid_trackers);
}

void addNewTrajectories(std::vector<Tracker> &trackers, const std::vector<Data>& frame, const std::vector<bool>& used, const std::vector<std::vector<float>>& knn_res, int initial_age, int n_states, float dt)
{
    //add new trajectories
    for (size_t i = 0; i < knn_res.size(); i++)
    {
        if (!used[i])
        {
            std::cout << "Adding a trajectory" << std::endl;
            trackers.push_back(Tracker(frame[knn_res[i][2]], initial_age, dt, n_states));
        }
    }
}

void Track(const std::vector<Data>& frame, float dt, int n_states, int initial_age, int age_threshold, std::vector<Tracker> &trackers)
{

    
    //delete trajectories not recently updated
    deleteOldTrajectories(trackers, age_threshold);

    //NN: associate new points with prevoius trajectories
    std::vector<Data> prev_trajs;
    for (auto t : trackers)
        prev_trajs.push_back(t.traj_.back());

    int n_cur_trajs = prev_trajs.size();
    std::vector<Data> new_trajs(n_cur_trajs);

    std::vector<std::vector<float>> knn_res = computeDistance(prev_trajs, frame);
    std::vector<bool> used(knn_res.size());
    std::vector<float> max_distance(n_cur_trajs, 50);

    for (size_t i = 0; i < trackers.size(); i++)
        trackers[i].age_--;

    
    
    int prev_i, cur_i;
    float dist;
    for (size_t i = 0; i < knn_res.size(); i++)
    {
        prev_i = knn_res[i][0];
        dist = knn_res[i][1];
        cur_i = knn_res[i][2];

        if (prev_i <= n_cur_trajs && dist < max_distance[prev_i])
        {
            trackers[prev_i].traj_.push_back(frame[cur_i]);

            new_trajs[prev_i] = frame[cur_i];
            max_distance[prev_i] = dist;
            trackers[prev_i].age_ += 1;
            used[i] = 1;
        }
    }
    

    //Kalman step
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_states, n_states);
    Eigen::VectorXf z(n_states);

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
            trackers[i].z_list_.push_back(State(new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0));
        }

        

        //trackers[i].ekf_.printInternalState();
        trackers[i].ekf_.EKFStep(H, z);
        
        trackers[i].pred_list_.push_back(trackers[i].ekf_.getEstimatedState());

        
    }

    
    //Add new trajectories from points from the current frame that were not associated
    addNewTrajectories(trackers, frame, used, knn_res, initial_age, n_states, dt);

    
}

void TrackOnGivenData(const std::vector<Data>& data, float dt, int n_states)
{
    int frame_id = 0;
    std::vector<Data> cur_frame;

    std::vector<Tracker> trackers;

    int initial_age = -5;
    int age_threshold = -10;

    std::cout << "Start" << std::endl;

    for (auto d : data)
    {
        if (d.frame_ != frame_id)
        {

            if (frame_id == 0) //initialization
            {
                for (auto o : cur_frame)
                {
                    trackers.push_back(Tracker(o, initial_age, dt, n_states));
                }
            }
            else //tracking
            {

                Track(cur_frame, dt, n_states, initial_age, age_threshold, trackers);
            }

            cur_frame.clear();
            frame_id = d.frame_;
        }
        cur_frame.push_back(d);
    }

    for (size_t i = 0; i < trackers.size(); i++)
        if (trackers[i].z_list_.size() > 10)
            plotTruthvsPred(trackers[i].z_list_, trackers[i].pred_list_);

    std::cout << "End." << std::endl;
}