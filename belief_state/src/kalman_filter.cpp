#include "kalman_filter.h"
#include <Eigen/Dense>

KalmanFilter::KalmanFilter(){
	this->val_intialized = this->mat_initialized = false;
}

void KalmanFilter::init(int x_size, int z_size){
	this->x_size = x_size;
	this->z_size = z_size;
	Eigen::MatrixXd I(x_size, x_size);
	I.setIdentity();
	this->I = I;
}

void KalmanFilter::init(const Eigen::VectorXd x0, const Eigen::MatrixXd P0){
	this->x = x0;
	this->P = P0;
	this->val_intialized = true;
}

void KalmanFilter::init(const Eigen::MatrixXd F, const Eigen::MatrixXd H, const Eigen::MatrixXd Q, const Eigen::MatrixXd R, double dt){
	this->dt = dt;
	this->F = F;
	this->H = H;
	this->Q = Q;
	this->R = R;
	this->mat_initialized = true;
}

void KalmanFilter::predict(){
	this->x_prior = (this->F)*(this->x);
	this->P_prior = (this->F)*(this->P)*(this->F.transpose()) + this->Q;
}

void KalmanFilter::predict(const Eigen::MatrixXd F, double dt){
	this->F = F;
	this->dt = dt;
	this->predict();	
}

Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd z){
	this->S = (this->H)*(this->P_prior)*(this->H.transpose()) + this->R;
	this->K = (this->P_prior)*(this->H.transpose())*(this->S.inverse());
	this->y = z - (this->H)*(this->x_prior);
	this->x_post = this->x_prior + (this->K)*(this->y);
	Eigen::MatrixXd I_KH;
	I_KH = this->I - (this->K*this->H);
	this->P_post = (I_KH)*(this->P_prior)*(I_KH.transpose()) + (this->K)*(this->R)*(this->K.transpose());
	this->x = this->x_post;
	this->P = this->P_post;
	return this->x;
}

bool KalmanFilter::intialized(){
	if(this->mat_initialized && this->val_intialized) return true;
	return false;
}
