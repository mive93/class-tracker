#include "ekf.h"

namespace tracking{
EKF::EKF() {}

EKF::EKF(const int n_states, const float dt_, const FMatrixF &Q_, const FMatrixF &R_, const state &in_state)
{
    nStates = n_states;
    dt = dt_;

    Q = Q_;
    R = R_;
    P = FMatrixF::Identity(nStates, nStates);
    H = FMatrixF::Identity(nStates, nStates);

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


void EKF::step(const FMatrixF &H_, const Eigen::VectorXf &z)
{
    H = H_;

    //predict
    state x = stateTransition();
    FMatrixF J = jacobian(x);
    FMatrixF PPred = J * P * J.transpose() + Q;

    //update

    FMatrixF Ht = H.transpose();

    Eigen::VectorXf y = z - StateIntoVector(xEst, nStates);
    FMatrixF S = H * PPred * Ht + R;
    FMatrixF K = PPred * Ht * (S.inverse());
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

FMatrixF EKF::jacobian(const state &x)
{
    FMatrixF J(nStates, nStates);
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

} //namespace tracking