#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  // error at each point along the path
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()|| estimations.size() == 0){
	  cout << "Invalid estimation or ground_truth data" << endl;
	  return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < int(estimations.size()); ++i){
	  // residual of each element
      VectorXd residual = estimations[i]-ground_truth[i];
      // sq error element-wise multiplication
      residual = residual.array()*residual.array();
      // element-wise summation
      rmse += residual;
      // rmse is still a vector
  }

  //calculate the mean
  rmse = rmse/estimations.size();
  //calculate the squared root (element wise)
  rmse = rmse.array().sqrt();
  return rmse;
}

void Tools::CalculateJacobian(const VectorXd &x_state, MatrixXd &H, VectorXd &h) {
  /**
  TODO:
    * Calculate a Jacobian and predicted measurement here.
  */
  // This function compute the maeasurement h and Jacobian H of Radar
  // recover state parameters
  bool is_zero_div = false;
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  // check division by zero from px*px + py*py
  float c1 = px*px + py*py;
  if(fabs(c1) < ZERO_TOL_){
   cout << "WARNING - Division by Zero" << endl;
   c1 = ZERO_TOL_;
   is_zero_div = true;
  }
  
  float c2 = sqrt(c1);
  float c3 = c1*c2;
  // compute the Jacobian matrix
  // note: not all elements have the boundary solution, so using ZERO_TOL instead
  //       for the Jacobian solution
  H  << (px/c2), (py/c2), 0, 0,
	   -(py/c1), (px/c1), 0, 0,
		 py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
		 
  // compute the measurement vector
  float rho = c2;
  // TODO atan2(0,0) is ok on my system but 
  //      Systems supporting symbolic mathematics normally return an 'undefined value' for atan2(0, 0)
  //      We need to handle this just to be sure everything running OK
  float theta = float(atan2(x_state(1), x_state(0)));
  float rho_dot; 
  if(is_zero_div){
	  //L'Hospital limit of 2 variables defined by px=rcos(theta), py=rsin(theta)
	  rho_dot = vx*cos(theta) + vy*sin(theta); 
  }
  else{
	  rho_dot = (px*vx + py*vy)/rho;
  }
  // predicted measurement
  h << rho, theta, rho_dot;
  
}







