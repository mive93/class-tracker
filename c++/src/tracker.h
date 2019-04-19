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
    std::vector<std::vector<float>> res;
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
        res.push_back(infos);
    }

    std::sort(res.begin(), res.end(), predicate);

    /* for(auto r: res)
    {
        for(auto i: r)
            std::cout<<i<<" ";
        std::cout<<std::endl;
    } */

    return res;
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
    ekf.printInternalState();
    return ekf;
}


void deleteOldTrajectories(std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &zList, std::vector<std::vector<State>> &predList, std::vector<EKF> &EKFList, std::vector<int> &trajectoryAge, int age_threshold)
{
    std::vector<std::vector<Data>> clean_trajectories;
    std::vector<std::vector<State>> clean_zList; 
    std::vector<std::vector<State>> clean_predList;
    std::vector<EKF> clean_EKFList;
    std::vector<int> clean_trajectoryAge;


    for(size_t i=0; i<trajectories.size(); i++)
    {
        if(trajectoryAge[i] > age_threshold)
        {
            clean_trajectories.push_back(trajectories[i]);
            clean_zList.push_back(zList[i]);
            clean_predList.push_back(predList[i]);
            clean_EKFList.push_back(EKFList[i]);
            clean_trajectoryAge.push_back(trajectoryAge[i]);
        }
        else
        {
            if(zList[i].size() > 10)
                plotTruthvsPred(zList[i], predList[i]);
        }
        

    }

    trajectories.swap(clean_trajectories);
    zList.swap(clean_zList);
    predList.swap(clean_predList);
    EKFList.swap(clean_EKFList);
    trajectoryAge.swap(clean_trajectoryAge);
}

void Track(std::vector<Data> frame, float dt, int n_states, int initial_age, int age_threshold, std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &zList, std::vector<std::vector<State>> &predList, std::vector<EKF> &EKFList, std::vector<int> trajectoryAge)
{

    deleteOldTrajectories(trajectories, zList, predList, EKFList, trajectoryAge, age_threshold);

    std::vector<Data> prev_trajs;
    for (auto t : trajectories)
        prev_trajs.push_back(t.back());

    int n_cur_trajs = prev_trajs.size();
    std::vector<Data> new_trajs(n_cur_trajs);

    std::vector<std::vector<float>> res = computeDistance(prev_trajs, frame);
    std::vector<bool> used(res.size());
    std::vector<float> max_distance(n_cur_trajs, 50);

    for (size_t i = 0; i < trajectoryAge.size(); i++)
        trajectoryAge[i]--;

    int prev_i, cur_i;
    float dist;
    for (size_t i = 0; i < res.size(); i++)
    {
        prev_i = res[i][0];
        dist = res[i][1];
        cur_i = res[i][2];

        if (prev_i <= n_cur_trajs && dist < max_distance[prev_i])
        {
            frame[cur_i].print();
            trajectories[prev_i].push_back(frame[cur_i]);

            new_trajs[prev_i] = frame[cur_i];
            max_distance[prev_i] = dist;
            trajectoryAge[prev_i] += 2;
            used[i] = 1;
        }
    }

    std::cout << "New traj" << std::endl;
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_states, n_states);
    Eigen::VectorXf z(n_states);

    for (size_t i = 0; i < new_trajs.size(); i++)
    {
        new_trajs[i].print();

        z << new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0;

        if (new_trajs[i].x_ == 0 && new_trajs[i].y_ == 0 && new_trajs[i].frame_ == -1)
        {
            std::cout << "No update" << std::endl;
            H(0, 0) = 0;
            H(1, 1) = 0;
        }
        else
        {
            std::cout << "Update" << std::endl;
            H(0, 0) = 1;
            H(1, 1) = 1;
            zList[i].push_back(State(new_trajs[i].x_, new_trajs[i].y_, 0, 0, 0));
        }

        std::cout << "EFK" << std::endl;
        EKFList[i].printInternalState();
        EKFList[i].EKFStep(H, z);
        std::cout << "Predlist" << std::endl;
        predList[i].push_back(EKFList[i].getEstimatedState());
    }

    for (auto pl : predList)
        for (auto s : pl)
            s.print();


    //add new trajectories
    std::vector<Data> first_point;
    for (size_t i = 0; i < res.size(); i++)
    {
        if (!used[i])
        {
            std::cout<<"Adding a trajctory"<<std::endl;
            first_point.push_back(frame[res[i][2]]);
            trajectories.push_back(first_point);
            EKFList.push_back(EFKinitialize(dt, n_states, frame[res[i][2]]));
            zList.resize(zList.size()+1);
            predList.resize(predList.size()+1);
            trajectoryAge.resize(trajectoryAge.size()+1);
            trajectoryAge.back() = initial_age;
        }
    } 
}



void TrackOnGivenData(std::vector<Data> data, float dt, int n_states)
{
    int frame_id = 0;
    std::vector<Data> cur_frame;

    std::vector<std::vector<Data>> trajectories;
    std::vector<std::vector<State>> zList;
    std::vector<std::vector<State>> predList;
    std::vector<EKF> EKFList;
    std::vector<int> trajectoryAge;

    int initial_age = -5;
    int age_threshold = -10;

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
                    EKFList.push_back(EFKinitialize(dt,n_states,o));
                    trajectoryAge.push_back(initial_age);
                }

                zList.resize(cur_frame.size());
                predList.resize(cur_frame.size());
            }
            else //tracking
            {

                Track(cur_frame, dt, n_states, initial_age, age_threshold, trajectories, zList, predList, EKFList, trajectoryAge);
                
            }

            cur_frame.clear();
            frame_id = d.frame_;
        }
        cur_frame.push_back(d);
    }

    for(int i=0; i<trajectories.size(); i++)
        if(zList[i].size() > 10)
            plotTruthvsPred(zList[i], predList[i]);

    std::cout << "finish" << std::endl;
}



#endif /*TRACKER_H*/