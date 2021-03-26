#include <iostream>
#include <eigen3/Eigen/Core>
#include "ukf.h"
#include "obj.h"
#ifdef USE_MATPLOTLIB
#include "plot.h"
#endif
#include "Tracking.h"
#include "filter.h"

void UKFtest()
{
    std::cout << "This is a test" << std::endl;

    int n_states = 5;
    float dt = 0.03;

    tracking::FMatrixF Q = tracking::FMatrixF::Zero(n_states, n_states);
    tracking::FMatrixF R = tracking::FMatrixF::Zero(n_states, n_states);

    Q.diagonal() << pow(1 * dt, 2), pow(1 * dt, 2), pow(1 * dt, 2), pow(3 * dt, 2), pow(0.1 * dt, 2);
    R.diagonal() << pow(0.5, 2), pow(0.5, 2), pow(0.1, 2), pow(0.5, 2), pow(0.02, 2);

    std::cout << Q << std::endl;
    std::cout << R << std::endl;

    tracking::state s(1, 2, 0, 0, 0);

    tracking::UKF ukf(n_states, dt, Q, R, s);
    ukf.printInternalState();

    tracking::FMatrixF H = tracking::FMatrixF::Zero(n_states, n_states);
    H(0, 0) = 1;
    H(1, 1) = 1;

    Eigen::VectorXf v(n_states);
    v << 3, 4, 0, 0, 0;

    ukf.step(H, v);
    ukf.printInternalState();
}

int main(int argc, char **argv)
{

    std::vector<tracking::obj_m> data = readDataFromFile("../data/test_ll.txt");
    for (auto d : data)
        d.print();

    // #ifdef USE_MATPLOTLIB
    //testMatplotlib();
    // #endif


    float dt = 0.03;
    int n_states = 5;
    int initial_age = 5;
    bool verbose = true;
    tracking::Filters_t filter_type = tracking::Filters_t::UKF_t;

    tracking::Tracking t(n_states, dt, initial_age, filter_type);
    t.trackOnGivenData(data, verbose);

    return EXIT_SUCCESS;
}
