#include "ukf.h"

namespace tracking{

void state::print()
{
    std::cout << "x: " << x << "\ty: " << y << "\tyaw: " << yaw << "\tv: " << vel << "\tyaw rate: " << yawRate << std::endl;
}

UKF::UKF() {}

UKF::UKF(const int n_states, const float dt_, const UKFMatrixF &Q_, const UKFMatrixF &R_, const state &in_state)
{
    nStates = n_states;
    dt = dt_;

    Q = Q_;
    R = R_;
    P = UKFMatrixF::Identity(nStates, nStates);
    H = UKFMatrixF::Identity(nStates, nStates);

    xEst = in_state;
}

UKF::~UKF()
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

void UKF::printInternalState()
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

void UKF::ukfStep(const UKFMatrixF &H_, const Eigen::VectorXf &z)
{
       H = H_;

    //predict
    //state x = stateTransition(); // x is predicted state
    // 1- compute sigma points
	float k = -2; //This is the hyper Parameter
	UKFMatrixF L(P.llt().matrixL()); //cholesky decomposition
	UKFMatrixF xhatsigmapoints(nStates, 2 * nStates + 1);
	
	UKFMatrixF xEstMatrix(nStates, 1); //converting xEst state to UKFMatrix type
	xEstMatrix << xEst.x, xEst.y, xEst.yaw, xEst.vel, xEst.yawRate; //converting xEst state to UKFMatrix type
	
	xhatsigmapoints.col(0) = xEstMatrix;
	int i;
	for (i = 1; i <= nStates; i++) {
		xhatsigmapoints.col(i) = xEstMatrix + sqrt(nStates + k) * L.col(i-1);
		xhatsigmapoints.col(nStates + i) = xEstMatrix - sqrt(nStates + k) * L.col(i-1);
	}
	//2-propagate sigma points
	UKFMatrixF xbrevesigmapoints(nStates, 2 * nStates + 1);
	for (i = 0; i < 2 * nStates + 1; i++) {
		state xinput(xhatsigmapoints(0, i), xhatsigmapoints(1, i), xhatsigmapoints(2, i), xhatsigmapoints(3, i), xhatsigmapoints(4, i));
		state x = stateTransition(xinput);
		UKFMatrixF xMatrix(nStates, 1);
		xMatrix << x.x, x.y, x.yaw, x.vel, x.yawRate;
		xbrevesigmapoints.col(i) = xMatrix;
	}
	//3-prediction
	UKFMatrixF alpha(2 * nStates + 1, 1);
	alpha(0) = k / (nStates + k);
	for (i = 1; i < 2 * nStates + 1; i++) {
		alpha(i) = 0.5 / (nStates + k);
	}
	UKFMatrixF xbreve(nStates, 1);
	xbreve = xbrevesigmapoints * alpha;
	UKFMatrixF pbreve(nStates, nStates);
	pbreve = UKFMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		pbreve += alpha(i) * (xbrevesigmapoints.col(i) - xbreve) * (xbrevesigmapoints.col(i) - xbreve).transpose();
	}
	pbreve += Q;
	//EKFMatrixF J = jacobian(x);
    //EKFMatrixF PPred = J * P * J.transpose() + Q;

    //update
	// 4.1 propagate sigma points for measurment function
	UKFMatrixF yhatsigmapoints(nStates, 2 * nStates + 1);
	yhatsigmapoints = H * xbrevesigmapoints;
	//4.2 mean and covariance of predicted measurment
	UKFMatrixF yhat(nStates, 1);
	yhat = yhatsigmapoints * alpha;
	UKFMatrixF py;
	py = UKFMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		py += alpha(i) * (yhatsigmapoints.col(i) - yhat) * (yhatsigmapoints.col(i) - yhat).transpose();
	}
	py += R;
	//4.3 cross covariance and Kalman gain
	UKFMatrixF pxy;
	pxy = UKFMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		pxy += alpha(i) * (xbrevesigmapoints.col(i) - xbreve) * (yhatsigmapoints.col(i) - yhat).transpose();
	}
	UKFMatrixF K;
	K = pxy * py.inverse();
	UKFMatrixF xhat;
	xhat = xbreve + K * (z - yhat);
	xEst.x = xhat(0);
	xEst.y = xhat(1);
	xEst.yaw = xhat(2);
	xEst.vel = xhat(3);
	xEst.yawRate = xhat(4);
	P = pbreve - K * py * K.transpose();

	//5 corrected mean and covariance

    //EKFMatrixF Ht = H.transpose();

    //Eigen::VectorXf y = z - StateIntoVector(xEst, nStates);
    //EKFMatrixF S = H * PPred * Ht + R;
    //EKFMatrixF K = PPred * Ht * (S.inverse());
    //Eigen::VectorXf x_est_vec = StateIntoVector(x, nStates) + K * y;
    //xEst = VectorIntoState(x_est_vec);
    //P = PPred - K * H * PPred;

}

state UKF::stateTransition(const state &x)
{
    //state x = xEst;
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



state UKF::getEstimatedState()
{
    return xEst;
}

} //namespace tracking
