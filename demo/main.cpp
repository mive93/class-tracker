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
    float dt = 0.03f;

    tracking::FMatrixF Q = tracking::FMatrixF::Zero(n_states, n_states);
    tracking::FMatrixF R = tracking::FMatrixF::Zero(n_states, n_states);
    float dt2 = dt * dt;
    Q.diagonal() << 1.0f*1.0f*dt2, 1.0f*1.0f*dt2, 1.0f*1.0f*dt2, 3.0f*3.0f*dt2, 0.1f*0.1f*dt2;
    R.diagonal() << 0.5f*0.5f, 0.5f*0.5f, 0.1f*0.1f, 0.5f*0.5f, 0.02f*0.02f;

    std::cout << Q << std::endl;
    std::cout << R << std::endl;

    tracking::state s(1.0f, 2.0f, 0.0f, 0.0f, 0.0f);

    tracking::UKF ukf(n_states, dt, Q, R, s);
    ukf.printInternalState();

    tracking::FMatrixF H = tracking::FMatrixF::Zero(n_states, n_states);
    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;

    Eigen::VectorXf v(n_states);
    v << 3.0f, 4.0f, 0.0f, 0.0f, 0.0f;

    ukf.step(H, v);
    ukf.printInternalState();
}

int main(/*int argc, char **argv*/)
{

    std::vector<tracking::obj_m> data = readDataFromFile("../data/test_ll.txt");
    for (auto d : data)
        d.print();

    // #ifdef USE_MATPLOTLIB
    //testMatplotlib();
    // #endif


    float dt = 0.03f;
    int n_states = 5;
    int initial_age = 5;
    bool verbose = true;
    tracking::Filters_t filter_type = tracking::Filters_t::EKF_t;

    tracking::Tracking t(n_states, dt, initial_age, filter_type);
    t.trackOnGivenData(data, verbose);

    return EXIT_SUCCESS;
}
