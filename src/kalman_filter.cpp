#include "kalman_filter.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  calculateNewEstimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = tools.Cartesian2Polar(x_);
  VectorXd y = z - z_pred;
  while(y[1] < -M_PI) {
    y[1] += 2*M_PI;
  }
  while(y[1] > M_PI) {
    y[1] -= 2*M_PI;
  }
  calculateNewEstimate(y);
}

void KalmanFilter::setInitialState(const VectorXd &measurement) {
  x_ << measurement[0],
        measurement[1],
        0,
        0;
}

void KalmanFilter::updateF(const float dt) {
  F_(0, 2) = dt;
  F_(1, 3) = dt;
}

void KalmanFilter::printState() {
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

void KalmanFilter::updateQ(const float dt) {
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
         0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
         dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
         0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
}

void KalmanFilter::setH_(const MatrixXd &H) {
  H_ = H;
}

void KalmanFilter::setJacobianFromState() {
  H_ = tools.CalculateJacobian(x_);
}

void KalmanFilter::setR_(const MatrixXd &R) {
  R_ = R;
}

VectorXd KalmanFilter::getState() {
  return x_;
}

void KalmanFilter::calculateNewEstimate(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}