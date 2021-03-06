#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
 private:
  // state vector
  Eigen::VectorXd x_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // state transition matrix
  Eigen::MatrixXd F_;
  // process covariance matrix
  Eigen::MatrixXd Q_;
  // measurement matrix
  Eigen::MatrixXd H_;
  // measurement covariance matrix
  Eigen::MatrixXd R_;
  // acceleration noise components
  const float noise_ax_ = 9.0;
  const float noise_ay_ = 9.0;
  void calculateNewEstimate(const Eigen::VectorXd &y); 
 public:
  KalmanFilter();
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  // sets the state from the first measurement
  void setInitialState(const Eigen::VectorXd &measurement);
  
  void updateF(const float dt);
  
  void updateQ(const float dt);
  
  void setH_(const Eigen::MatrixXd &H);
  
  void setR_(const Eigen::MatrixXd &R);
  
  void setJacobianFromState();
  
  Eigen::VectorXd getState();
  
  void printState();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
};

#endif // KALMAN_FILTER_H_
