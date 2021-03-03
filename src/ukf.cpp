#include "ukf.h"

namespace tracking{

UKF::UKF() {}

UKF::UKF(const int n_states, const float dt_, const FMatrixF &Q_, const FMatrixF &R_, const state &in_state)
{
    nStates = n_states;
    dt = dt_;

    Q = Q_;
    R = R_;
    P = FMatrixF::Identity(nStates, nStates);
    H = FMatrixF::Identity(nStates, nStates);

    xEst = in_state;
}

UKF::~UKF(){}


void UKF::step(const FMatrixF &H_, const Eigen::VectorXf &z)
{
       H = H_;

    //predict
    // 1- compute sigma points
	float k = -2; //This is the hyper Parameter
	FMatrixF L(P.llt().matrixL()); //cholesky decomposition
	FMatrixF xhatsigmapoints(nStates, 2 * nStates + 1);
	
	FMatrixF xEstMatrix(nStates, 1); //converting xEst state to UKFMatrix type
	xEstMatrix << xEst.x, xEst.y, xEst.yaw, xEst.vel, xEst.yawRate; //converting xEst state to UKFMatrix type
	
	xhatsigmapoints.col(0) = xEstMatrix;
	int i;
	for (i = 1; i <= nStates; i++) {
		xhatsigmapoints.col(i) = xEstMatrix + sqrt(nStates + k) * L.col(i-1);
		xhatsigmapoints.col(nStates + i) = xEstMatrix - sqrt(nStates + k) * L.col(i-1);
	}
	//2-propagate sigma points
	FMatrixF xbrevesigmapoints(nStates, 2 * nStates + 1);
	for (i = 0; i < 2 * nStates + 1; i++) {
		state xinput(xhatsigmapoints(0, i), xhatsigmapoints(1, i), xhatsigmapoints(2, i), xhatsigmapoints(3, i), xhatsigmapoints(4, i));
		state x = stateTransition(xinput);
		FMatrixF xMatrix(nStates, 1);
		xMatrix << x.x, x.y, x.yaw, x.vel, x.yawRate;
		xbrevesigmapoints.col(i) = xMatrix;
	}
	//3-prediction
	FMatrixF alpha(2 * nStates + 1, 1);
	alpha(0) = k / (nStates + k);
	for (i = 1; i < 2 * nStates + 1; i++) {
		alpha(i) = 0.5 / (nStates + k);
	}
	FMatrixF xbreve(nStates, 1);
	xbreve = xbrevesigmapoints * alpha;
	FMatrixF pbreve(nStates, nStates);
	pbreve = FMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		pbreve += alpha(i) * (xbrevesigmapoints.col(i) - xbreve) * (xbrevesigmapoints.col(i) - xbreve).transpose();
	}
	pbreve += Q;

    //update
	// 4.1 propagate sigma points for measurment function
	FMatrixF yhatsigmapoints(nStates, 2 * nStates + 1);
	yhatsigmapoints = H * xbrevesigmapoints;
	//4.2 mean and covariance of predicted measurment
	FMatrixF yhat(nStates, 1);
	yhat = yhatsigmapoints * alpha;
	FMatrixF py;
	py = FMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		py += alpha(i) * (yhatsigmapoints.col(i) - yhat) * (yhatsigmapoints.col(i) - yhat).transpose();
	}
	py += R;
	//4.3 cross covariance and Kalman gain
	FMatrixF pxy;
	pxy = FMatrixF::Zero(nStates, nStates);
	for (i = 0; i < 2 * nStates + 1; i++) {
		pxy += alpha(i) * (xbrevesigmapoints.col(i) - xbreve) * (yhatsigmapoints.col(i) - yhat).transpose();
	}
	FMatrixF K;
	K = pxy * py.inverse();
	FMatrixF xhat;
	xhat = xbreve + K * (z - yhat);
	xEst.x = xhat(0);
	xEst.y = xhat(1);
	xEst.yaw = xhat(2);
	xEst.vel = xhat(3);
	xEst.yawRate = xhat(4);
	P = pbreve - K * py * K.transpose();

	//5 corrected mean and covariance
}

state UKF::stateTransition(const state &x)
{
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


} //namespace tracking
