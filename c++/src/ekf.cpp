#include "ekf.h"

namespace tracking{

void state::print()
{
    std::cout << "x: " << x << "\ty: " << y << "\tyaw: " << yaw << "\tv: " << vel << "\tyaw rate: " << yawRate << std::endl;
}

EKF::EKF() {}

EKF::EKF(const int n_states, const float dt_, const EKFMatrixF &Q_, const EKFMatrixF &R_, const state &in_state)
{
    nStates = n_states;
    dt = dt_;

    Q = Q_;
    R = R_;
    P = EKFMatrixF::Identity(nStates, nStates);
    H = EKFMatrixF::Identity(nStates, nStates);

    xEst = in_state;
}

EKF::~EKF()
{
    //FIXME - undestand why segfault
    /* if (Q)
        delete Q;
    if (R)
        delete R;
    if (H)
        delete H;
    if (P)
        delete P; */
}

void EKF::printInternalState()
{
    std::cout << "***************Internal state*********************" << std::endl;
    std::cout << "n_states: " << nStates << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "P: \n"
              << P << std::endl;
    std::cout << "Q: \n"
              << Q << std::endl;
    std::cout << "R: \n"
              << R << std::endl;
    std::cout << "H: \n"
              << H << std::endl;
    xEst.print();
    std::cout << "**************************************************" << std::endl;
}

Eigen::VectorXf StateIntoVector(const state &x, int n_states)
{
    Eigen::VectorXf v(n_states);
    v(0) = x.x;
    v(1) = x.y;
    v(2) = x.yaw;
    v(3) = x.vel;
    v(4) = x.yawRate;
    return v;
}

state VectorIntoState(const Eigen::VectorXf &v)
{
    state x(v(0), v(1), v(2), v(3), v(4));
    return x;
}

void EKF::ekfStep(const EKFMatrixF &H_, const Eigen::VectorXf &z)
{
    H = H_;

    //predict
    state x = stateTransition();
    EKFMatrixF J = jacobian(x);
    EKFMatrixF PPred = J * P * J.transpose() + Q;

    //update

    EKFMatrixF Ht = H.transpose();

    Eigen::VectorXf y = z - StateIntoVector(xEst, nStates);
    EKFMatrixF S = H * PPred * Ht + R;
    EKFMatrixF K = PPred * Ht * (S.inverse());
    Eigen::VectorXf x_est_vec = StateIntoVector(x, nStates) + K * y;
    xEst = VectorIntoState(x_est_vec);
    P = PPred - K * H * PPred;
}

state EKF::stateTransition()
{
    state x = xEst;
    state y;
    if (abs(x.yawRate) < 0.0001){
        y.x = x.x + x.vel * dt * cos(x.yaw);
        y.y = x.y + x.vel * dt * sin(x.yaw);
        y.yaw = x.yaw;
        y.vel = x.vel;
        y.yawRate = 0.0001;
    }
    else{
        y.x = x.x + (x.vel / x.yawRate) * (sin(x.yawRate * dt + x.yaw) - sin(x.yaw));
        y.y = x.y + (x.vel / x.yawRate) * (-cos(x.yawRate * dt + x.yaw) + cos(x.yaw));
        y.yaw = x.yaw + x.yawRate * dt;
        y.vel = x.vel;
        y.yawRate = x.yawRate;
    }

    return y;
}

EKF::EKFMatrixF EKF::jacobian(const state &x)
{
    EKFMatrixF J(nStates, nStates);
    J.setIdentity();

    J(0, 2) = (x.vel / x.yawRate) * (cos(x.yawRate * dt + x.yaw) - cos(x.yaw));
    J(0, 3) = (1.0 / x.yawRate) * (sin(x.yawRate * dt + x.yaw) - sin(x.yaw));
    J(0, 4) = (dt * x.vel / x.yawRate) * cos(x.yawRate * dt + x.yaw) - (x.vel / pow(x.yawRate, 2)) * (sin(x.yawRate * dt + x.yaw) - sin(x.yaw));
    J(1, 2) = (x.vel / x.yawRate) * (sin(x.yawRate * dt + x.yaw) - sin(x.yaw));
    J(1, 3) = (1.0 / x.yawRate) * (-cos(x.yawRate * dt + x.yaw) + cos(x.yaw));
    J(1, 4) = (dt * x.vel / x.yawRate) * sin(x.yawRate * dt + x.yaw) - (x.vel / pow(x.yawRate, 2)) * (-cos(x.yawRate * dt + x.yaw) + cos(x.yaw));
    J(2, 4) = dt;

    return J;
}

state EKF::getEstimatedState()
{
    return xEst;
}

} //namespace tracking