#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  CalculateKalmanGain(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  float range = sqrt(pow(x_(0), 2) + pow(x_(1), 2));

  if (fabs(x_(0)) < 0.0001 || fabs(range) < 0.0001) {
    return;
  }
  float phi = atan2(x_(1), x_(0));
  VectorXd z_pred = VectorXd(3);
  z_pred << range, phi, (x_(0) * x_(2) + x_(1) * x_(3)) / range;
  VectorXd y = z - z_pred;

  if ( abs(y[1]) > M_PI ) {
    Tools tools;
    y[1] = tools.NormalizeAngle(y[1]);
  }
  CalculateKalmanGain(y);

}

void KalmanFilter::CalculateKalmanGain(const VectorXd &y){

  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
