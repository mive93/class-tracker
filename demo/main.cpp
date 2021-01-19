#include <iostream>
#include <eigen3/Eigen/Core>
#include "ekf.h"
#include "obj.h"
#include "plot.h"
#include "Tracking.h"

void EKFtest()
{
    std::cout << "This is a test" << std::endl;

    int n_states = 5;
    float dt = 0.03;

    tracking::EKF::EKFMatrixF Q = tracking::EKF::EKFMatrixF::Zero(n_states, n_states);
    tracking::EKF::EKFMatrixF R = tracking::EKF::EKFMatrixF::Zero(n_states, n_states);

    Q.diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);

    std::cout << Q << std::endl;
    std::cout << R << std::endl;

    tracking::state s(1, 2, 0, 0, 0);

    tracking::EKF ekf(n_states, dt, Q, R, s);
    ekf.printInternalState();

    tracking::EKF::EKFMatrixF H = tracking::EKF::EKFMatrixF::Zero(n_states, n_states);
    H(0, 0) = 1;
    H(1, 1) = 1;

    Eigen::VectorXf v(n_states);
    v << 3, 4, 0, 0, 0;

    ekf.ekfStep(H, v);
    ekf.printInternalState();
}

int main(int argc, char **argv)
{

    std::vector<tracking::obj_m> data = readDataFromFile("../data/test_ll.txt");
    for (auto d : data)
        d.print();

    //testMatplotlib();


    float dt = 0.03;
    int n_states = 5;
    int initial_age = 5;
    bool verbose = true;

    tracking::Tracking t(n_states, dt, initial_age);
    t.trackOnGivenData(data, verbose);

    return EXIT_SUCCESS;
}
