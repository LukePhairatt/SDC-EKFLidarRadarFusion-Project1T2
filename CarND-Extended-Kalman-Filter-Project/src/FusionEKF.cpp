#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>       



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  // see tool.cpp for non-linear radar measurement function
  Hj_ = MatrixXd(3, 4);
  hj_ = VectorXd(3);

  // measurement covariance matrix - laser (px,py variance)
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar (rho, phi, rho_dot variance)
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  // input process measurement noise due to acceleration in x,y
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
  // Lidar measurement transition- Fixed
  H_laser_ <<  1, 0, 0, 0,
			   0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // intial state vector from first measurement	
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ix = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float iy = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]); 
      ekf_.x_ << ix,iy,0,0;
      
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0; 
      
    }
    
    // intial state transition		   
	ekf_.F_ = MatrixXd::Identity(4, 4);
	
	// initial covariance matrix		  
	ekf_.Q_ = MatrixXd(4, 4);	
	ekf_.Q_ <<  0, 0, 0, 0,
			    0, 0, 0, 0,
			    0, 0, 0, 0,
			    0, 0, 0, 0; 	  
			   
	ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1000, 0,
			   0, 0, 0, 1000;		   
			   
    // init time stamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   
  // time lapsed
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  // Update F - state transition
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  // Update Q - process noise covariance
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
			  0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
			  dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
			  0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_; 
			  
  // Call prediction
  ekf_.Predict();
  

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    tools.CalculateJacobian(ekf_.x_, Hj_, hj_);
    ekf_.H_ = Hj_;
    ekf_.h_ = hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
