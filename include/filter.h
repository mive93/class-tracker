#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>

#include "state.h"

namespace tracking{

using FMatrixF = Eigen::Matrix<float, -1, -1>;
enum Filters_t {EKF_t, UKF_t};

class Filter{

    public:
    
    Filter(){};
    virtual ~Filter() = default;
    void printInternalState();
    state getEstimatedState();
    FMatrixF getCovarianceMatrix();
    virtual void step(const FMatrixF &H_, const Eigen::VectorXf &z) = 0;
    
    protected:

    int nStates;
    float dt;
    state xEst;
    FMatrixF Q;
    FMatrixF R;
    FMatrixF P;
    FMatrixF H;

};
}

#endif /*FILTER_H*/
