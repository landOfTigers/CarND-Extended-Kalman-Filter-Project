#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  
  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  
  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // state covariance matrix
  MatrixXd P = MatrixXd(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 100000, 0,
       0, 0, 0, 100000;
  
  // the initial transition matrix
  MatrixXd F = MatrixXd(4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  // process covariance matrix
  MatrixXd Q = MatrixXd(4, 4);
  Q << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0; 
  
  VectorXd x = VectorXd(4);
  x << 1, 1, 1, 1;
  
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);   
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      VectorXd cartesian = Tools::Polar2Cartesian(measurement_pack.raw_measurements_);
      ekf_.setInitialState(cartesian);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.setInitialState(measurement_pack.raw_measurements_);
    }
	previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
 
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.updateF(dt);

  // update the process covariance matrix
  ekf_.updateQ(dt);

  ekf_.Predict();

  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.setJacobianFromState();
    ekf_.setR_(R_radar_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.setH_(H_laser_);
    ekf_.setR_(R_laser_);
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  ekf_.printState();
}
