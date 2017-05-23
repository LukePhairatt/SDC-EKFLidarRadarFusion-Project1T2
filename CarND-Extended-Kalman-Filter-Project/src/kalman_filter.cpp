#include "kalman_filter.h"
#include <math.h>
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  // for both Lidar and Radar system
  // state vector (we moodel the input (accela) as noise in Q_)
  x_ = F_ * x_;
  // state covariance
  P_ = F_ * P_ * F_.transpose() + Q_;
  
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    * z =: px, py measurement
  */
  // measurement update- Lidar (H_ must be updated beforehand)
  VectorXd y = z - H_ * x_;
  // state update with the current innovation
  KFStateEstimate(y);
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    * z := range, angle, range_velocity
  */
  // measurement update- Radar (H_ must be updated beforehand)
  // innovation 
  VectorXd y = z - h_;
  // control -pi,+pi angle
  y(1) = fmod(y(1) + PI, 2*PI) - PI;
  // state update with the current innovation
  KFStateEstimate(y);
 
}

void KalmanFilter::KFStateEstimate(const VectorXd &y){	
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}





