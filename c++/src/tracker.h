#ifndef TRACKER_H
#define TRACKER_H

#include "ekf.h"
#include "utils.h"
#include "plot.h"

void Track(std::vector<Data> frame, float dt, int n_states, std::vector<std::vector<Data>> &trajectories, std::vector<std::vector<State>> &zList, std::vector<std::vector<State>> &predList, std::vector<EKF> &EKFList)
{
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
                Eigen::MatrixXf *Q = new Eigen::MatrixXf(n_states, n_states);
                Eigen::MatrixXf *R = new Eigen::MatrixXf(n_states, n_states);
                (*Q).diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
                (*R).diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);

                for (auto o : cur_frame)
                {
                    first_point.clear();
                    first_point.push_back(o);
                    trajectories.push_back(first_point);

                    State s(o.x_, o.y_, 0, 0, 0);
                    EKF ekf(n_states, dt, Q, R, s);
                    ekf.printInternalState();
                    EKFList.push_back(ekf);
                    trajectoryAge.push_back(initial_age);
                }
            }
            else //tracking
            {
                break;
                Track(cur_frame, dt, n_states, trajectories, zList, predList, EKFList);
            }

            cur_frame.clear();
            frame_id = d.frame_;
        }
        cur_frame.push_back(d);
    }
}


void deleteOldTrajectories();

#endif /*TRACKER_H*/