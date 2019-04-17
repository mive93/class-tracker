#include "ekf.h"


State::State()
{
    x_ = 0;
    y_ = 0;
    yaw_ = 0;
    vel_ = 0;
    yaw_rate_ = 0;
}

State::State(float x, float y, float yaw, float vel, float yaw_rate)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    vel_ = vel;
    yaw_rate_ = yaw_rate;
}

void State::PrintState()
{
    std::cout << x_ << " " << y_ << " " << yaw_ << " " << vel_ << " " << yaw_rate_ << std::endl;
}

EKF::EKF(int n_states, float dt, Eigen::MatrixXf *Q, Eigen::MatrixXf *R, State in_state)
{

    n_states_ = n_states;
    dt_ = dt;

    Q_ = new Eigen::MatrixXf(n_states, n_states);
    R_ = new Eigen::MatrixXf(n_states, n_states);
    H_ = new Eigen::MatrixXf(n_states, n_states);
    P_ = new Eigen::MatrixXf(n_states, n_states);

    *Q_ = *Q;
    *R_ = *R;
    (*P_).setIdentity();
    (*H_).setZero();

    x_est_ = in_state;
}

void EKF::printInternalState()
{
    std::cout << "n_states: " << n_states_ << std::endl;
    std::cout << "dt_: " << dt_ << std::endl;
    std::cout << "P: \n"
              << *P_ << std::endl;
    std::cout << "Q: \n"
              << *Q_ << std::endl;
    std::cout << "R: \n"
              << *R_ << std::endl;
    std::cout << "H: \n"
              << *H_ << std::endl;
    x_est_.PrintState();
}

Eigen::VectorXf StateIntoVector(State x, int n_states)
{
    Eigen::VectorXf v(n_states);
    v(0) = x.x_;
    v(1) = x.y_;
    v(2) = x.yaw_;
    v(3) = x.vel_;
    v(4) = x.yaw_rate_;
    return v;
}

State VectorIntoState(Eigen::VectorXf v)
{
    State x(v(0), v(1), v(2), v(3), v(4));
    return x;
}

void EKF::EKFStep(Eigen::MatrixXf H, Eigen::VectorXf z)
{
    *H_ = H;

    //predict
    State x = StateTransition();
    Eigen::MatrixXf J = Jacobian(x);
    Eigen::MatrixXf PPred = J * (*P_) * J.transpose() + (*Q_);

    //update
    Eigen::VectorXf y = z - StateIntoVector(x_est_, n_states_);
    Eigen::MatrixXf S = (*H_) * PPred * (*H_).transpose() + (*R_);
    Eigen::MatrixXf K = PPred * (*H_).transpose() * (S.inverse());
    Eigen::VectorXf x_est_vec = StateIntoVector(x, n_states_) + K * y;
    x_est_ = VectorIntoState(x_est_vec);
    (*P_) = PPred - K * (*H_) * PPred;
}

State EKF::StateTransition()
{
    State x = x_est_;
    State y;
    if (abs(x.yaw_rate_) < 0.0001)
    {
        y.x_ = x.x_ + x.vel_ * dt_ * cos(x.yaw_);
        y.y_ = x.y_ + x.vel_ * dt_ * sin(x.yaw_);
        y.yaw_ = x.yaw_;
        y.vel_ = x.vel_;
        y.yaw_rate_ = 0.0001;
    }
    else
    {
        y.x_ = x.x_ + (x.vel_ / x.yaw_rate_) * (sin(x.yaw_rate_ * dt_ * x.yaw_ - sin(x.yaw_)));
        y.y_ = x.y_ + (x.vel_ / x.yaw_rate_) * (-cos(x.yaw_rate_ * dt_ * x.yaw_ + cos(x.yaw_)));
        y.y_ = x.y_ + x.vel_ * dt_ * sin(x.yaw_);
        y.yaw_ = x.yaw_ + x.yaw_rate_ * dt_;
        y.vel_ = x.vel_;
        y.yaw_rate_ = x.yaw_rate_;
    }

    return y;
}

Eigen::MatrixXf EKF::Jacobian(State x)
{

    Eigen::MatrixXf J(n_states_, n_states_);
    J.setIdentity();

    J(0, 2) = (x.vel_ / x.yaw_rate_) * (cos(x.yaw_rate_ * dt_ + x.yaw_) - cos(x.yaw_));
    J(0, 3) = (1.0 / x.yaw_rate_) * (sin(x.yaw_rate_ * dt_ + x.yaw_) - sin(x.yaw_));
    J(0, 4) = (dt_ * x.vel_ / x.yaw_rate_) * cos(x.yaw_rate_ * dt_ + x.yaw_) - (x.vel_ / pow(x.yaw_rate_, 2)) * (sin(x.yaw_rate_ * dt_ + x.yaw_) - sin(x.yaw_));
    J(1, 2) = (x.vel_ / x.yaw_rate_) * (sin(x.yaw_rate_ * dt_ + x.yaw_) - sin(x.yaw_));
    J(1, 3) = (1.0 / x.yaw_rate_) * (-cos(x.yaw_rate_ * dt_ + x.yaw_) + cos(x.yaw_));
    J(1, 4) = (dt_ * x.vel_ / x.yaw_rate_) * sin(x.yaw_rate_ * dt_ + x.yaw_) - (x.vel_ / pow(x.yaw_rate_, 2)) * (-cos(x.yaw_rate_ * dt_ + x.yaw_) + cos(x.yaw_));
    J(2, 4) = dt_;

    return J;
}
