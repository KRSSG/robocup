#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter{

	//state and measurment dimensions
	int x_size, z_size;

	//transition, measurment, process noise, measurment noise, covariance, system uncertainity, kalman gain, identity
	Eigen::MatrixXd F, H, Q, R, P, P_prior, P_post, S, K, I;

	//current state, residual
	Eigen::VectorXd x, x_prior, x_post, y;

	//time step
	double dt;

	//intialization
	bool val_intialized, mat_initialized;

public:
	KalmanFilter();
	void init(int x_size, int z_size);
	void init(const Eigen::VectorXd x0, const Eigen::MatrixXd P0);
	void init(const Eigen::MatrixXd F, const Eigen::MatrixXd H, const Eigen::MatrixXd Q, const Eigen::MatrixXd R, double dt);
	void predict();
	void predict(const Eigen::MatrixXd F, double dt);
	bool intialized();
	Eigen::VectorXd update(Eigen::VectorXd z);

};

#endif