#include <iostream>
#include <eigen3/Eigen/Core>
#include "ekf.h"
#include "trackutils.h"
#include "plot.h"
#include "tracker.h"

#include <flann/flann.h>

void EKFtest()
{
    std::cout << "This is a test" << std::endl;

    int n_states = 5;
    float dt = 0.03;

    EKF::EKFMatrixF Q = EKF::EKFMatrixF::Zero(n_states, n_states);
    EKF::EKFMatrixF R = EKF::EKFMatrixF::Zero(n_states, n_states);

    Q.diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);

    std::cout << Q << std::endl;
    std::cout << R << std::endl;

    State s(1, 2, 0, 0, 0);

    EKF ekf(n_states, dt, Q, R, s);
    ekf.printInternalState();

    EKF::EKFMatrixF H = EKF::EKFMatrixF::Zero(n_states, n_states);
    H(0, 0) = 1;
    H(1, 1) = 1;

    Eigen::VectorXf v(n_states);
    v << 3, 4, 0, 0, 0;

    ekf.EKFStep(H, v);
    ekf.printInternalState();
}

int main(int argc, char **argv)
{

    std::vector<Data> data = readDataFromFile("../../data/test_ll.txt");
    for (auto d : data)
        d.print();

    //testMatplotlib();

    float dt = 0.03;
    int n_states = 5;
    TrackOnGivenData(data, dt, n_states, true);

    return EXIT_SUCCESS;
}
