#include "tools.h"
#include <iostream>
#include <math.h> 

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() != ground_truth.size() || estimations.empty()) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  for (unsigned int i=0; i < estimations.size(); ++i) {    
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  rmse /= estimations.size();
  return rmse.array().sqrt();  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  MatrixXd jacobian(3,4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian() error: Division by zero!" << endl;
    return jacobian;
  }

  jacobian << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return jacobian;
}

VectorXd Tools::Cartesian2Polar(const VectorXd& x_state) {

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  VectorXd polar(3);
  float c = sqrt(px*px+py*py);
  if (fabs(c) < 0.0001 || fabs(px) < 0.0001) {
    cout << "Cartesian2Polar() error: Division by zero!" << endl;
    return polar;
  }
  
  polar << c, atan2(py, px), (px*vx+py*vy)/c;

  return polar;
}

VectorXd Tools::Polar2Cartesian(const VectorXd& measurement) {

  float rho = measurement(0);
  float phi = measurement(1);
  
  VectorXd cartesian(2);
  cartesian << rho*cos(phi), rho*sin(phi);

  return cartesian;
}
