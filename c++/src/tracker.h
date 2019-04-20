#ifndef TRACKER_H
#define TRACKER_H

#include "ekf.h"
#include "utils.h"
#include "plot.h"

bool predicate(const std::vector<float> &a, const std::vector<float> &b)
{
    return (a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]));
}

std::vector<std::vector<float>> computeDistance(std::vector<Data> old_points, std::vector<Data> new_points)
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

EKF EFKinitialize(float dt, int n_states, Data first_point)
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

void deleteOldTrajectories(std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &z_list, std::vector<std::vector<State>> &pred_list, std::vector<EKF> &EKF_list, std::vector<int> &trajectory_age, int age_threshold)
{
    std::vector<std::vector<Data>> clean_trajectories;
    std::vector<std::vector<State>> clean_z_list;
    std::vector<std::vector<State>> clean_pred_list;
    std::vector<EKF> clean_EKF_list;
    std::vector<int> clean_trajectory_age;

    for (size_t i = 0; i < trajectories.size(); i++)
    {
        if (trajectory_age[i] > age_threshold)
        {
            clean_trajectories.push_back(trajectories[i]);
            clean_z_list.push_back(z_list[i]);
            clean_pred_list.push_back(pred_list[i]);
            clean_EKF_list.push_back(EKF_list[i]);
            clean_trajectory_age.push_back(trajectory_age[i]);
        }
        else
        {
            std::cout << "Deleting a trajctory" << std::endl;
            if (z_list[i].size() > 10)
                plotTruthvsPred(z_list[i], pred_list[i]);
        }
    }

    trajectories.swap(clean_trajectories);
    z_list.swap(clean_z_list);
    pred_list.swap(clean_pred_list);
    EKF_list.swap(clean_EKF_list);
    trajectory_age.swap(clean_trajectory_age);
}

void addNewTrajectories(std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &z_list, std::vector<std::vector<State>> &pred_list, std::vector<EKF> &EKF_list, std::vector<int> &trajectory_age, std::vector<Data> frame,std::vector<bool> used, std::vector<std::vector<float>> knn_res,int initial_age, int n_states, float dt)
{

    //add new trajectories
    std::vector<Data> first_point;
    for (size_t i = 0; i < knn_res.size(); i++)
    {
        if (!used[i])
        {
            std::cout << "Adding a trajctory" << std::endl;
            first_point.push_back(frame[knn_res[i][2]]);
            trajectories.push_back(first_point);
            EKF_list.push_back(EFKinitialize(dt, n_states, frame[knn_res[i][2]]));
            z_list.resize(z_list.size() + 1);
            pred_list.resize(pred_list.size() + 1);
            trajectory_age.resize(trajectory_age.size() + 1);
            trajectory_age.back() = initial_age;
        }
    }
}

void Track(std::vector<Data> frame, float dt, int n_states, int initial_age, int age_threshold, std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &z_list, std::vector<std::vector<State>> &pred_list, std::vector<EKF> &EKF_list, std::vector<int> trajectory_age)
{

    //delete trajectories not recently updated
    deleteOldTrajectories(trajectories, z_list, pred_list, EKF_list, trajectory_age, age_threshold);

    //NN: associate new points with prevoius trajectories
    std::vector<Data> prev_trajs;
    for (auto t : trajectories)
        prev_trajs.push_back(t.back());

    int n_cur_trajs = prev_trajs.size();
    std::vector<Data> new_trajs(n_cur_trajs);

    std::vector<std::vector<float>> knn_res = computeDistance(prev_trajs, frame);
    std::vector<bool> used(knn_res.size());
    std::vector<float> max_distance(n_cur_trajs, 50);

    for (size_t i = 0; i < trajectory_age.size(); i++)
        trajectory_age[i]--;

    int prev_i, cur_i;
    float dist;
    for (size_t i = 0; i < knn_res.size(); i++)
    {
        prev_i = knn_res[i][0];
        dist = knn_res[i][1];
        cur_i = knn_res[i][2];

        if (prev_i <= n_cur_trajs && dist < max_distance[prev_i])
        {
            frame[cur_i].print();
            trajectories[prev_i].push_back(frame[cur_i]);

            new_trajs[prev_i] = frame[cur_i];
            max_distance[prev_i] = dist;
            trajectory_age[prev_i] += 2;
            used[i] = 1;
        }
    }

    //Kalman step
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_states, n_states);
    Eigen::VectorXf z(n_states);

    for (size_t i = 0; i < new_trajs.size(); i++)
    {
        new_trajs[i].print();

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
            z_list[i].push_back(State(new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0));
        }

        EKF_list[i].printInternalState();
        EKF_list[i].EKFStep(H, z);
        pred_list[i].push_back(EKF_list[i].getEstimatedState());
    }

    /* for (auto pl : pred_list)
        for (auto s : pl)
            s.print();*/

    //Add new trajectories from points from the current frame that were not associated
    addNewTrajectories(trajectories, z_list, pred_list, EKF_list, trajectory_age, frame,used, knn_res,initial_age, n_states, dt);

    
}

void TrackOnGivenData(std::vector<Data> data, float dt, int n_states)
{
    int frame_id = 0;
    std::vector<Data> cur_frame;

    std::vector<std::vector<Data>> trajectories;
    std::vector<std::vector<State>> z_list;
    std::vector<std::vector<State>> pred_list;
    std::vector<EKF> EKF_list;
    std::vector<int> trajectory_age;

    int initial_age = -5;
    int age_threshold = -10;

    std::cout << "Start" << std::endl;

    for (auto d : data)
    {
        if (d.frame_ != frame_id)
        {

            if (frame_id == 0) //initialization
            {
                std::vector<Data> first_point;
                for (auto o : cur_frame)
                {
                    first_point.clear();
                    first_point.push_back(o);
                    trajectories.push_back(first_point);
                    EKF_list.push_back(EFKinitialize(dt, n_states, o));
                    trajectory_age.push_back(initial_age);
                }

                z_list.resize(cur_frame.size());
                pred_list.resize(cur_frame.size());
            }
            else //tracking
            {

                Track(cur_frame, dt, n_states, initial_age, age_threshold, trajectories, z_list, pred_list, EKF_list, trajectory_age);
            }

            cur_frame.clear();
            frame_id = d.frame_;
        }
        cur_frame.push_back(d);
    }

    for (size_t i = 0; i < trajectories.size(); i++)
        if (z_list[i].size() > 10)
            plotTruthvsPred(z_list[i], pred_list[i]);

    std::cout << "End." << std::endl;
}

#endif /*TRACKER_H*/