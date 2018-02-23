#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  R_radar_ = MatrixXd(3, 3);

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;


  // measurement matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  // measurement matrix - rader
  Hj_ = MatrixXd(3, 4);


  // state transition matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // state covariance matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // process covariance matrix
  Q_ = MatrixXd(4, 4);
  Q_ = 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0;

  noise_ax = 9;
  noise_ay = 9;
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

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      // Get the raw measurements in polar coordinates
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      // Convert polar coordinates to cartersian coordinates
      VectorXd(4) input;
      input(0) = ro * cos(theta); //px
      input(1) = ro * sin(theta); //py
      input(2) = ro_dot * cos(theta); //vx
      input(3) = ro_dot * sin(theta); //vy

      // Initialize EKF
      ekf_.Init(input, P_, F_, Hj, R_radar_, Q_); 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      // Get the raw measurements
      VectorXd(4) input;
      input(0) = measurement_pack.raw_measurements_(0); //px
      input(1) = measurement_pack.raw_measurements_(1); //py
      input(2) = 0; //vx
      input(3) = 0; //vy

      // Initialize EKF
      ekf_.Init(input, P_, F_, H_laser_, R_laser_, Q_); 
    }

    // store the starting timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Delta_T in seconds
  float delta_time = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  // Updating the state transition matrix
  ekf_.F_(0, 2) = delta_time;
  ekf_.F_(1, 3) = delta_time;

  float delta_t_squared = (delta_time * delta_time);
  float delta_t_cubed = (delta_t_squared * delta_time);
  float delta_t_quadrupled = (delta_t_squared * delta_t_squared);

  // Updating the process noise covariance matrix
  ekf_.Q_ << (delta_t_quadrupled * noise_ax / 4), 0, (delta_t_cubed * noise_ax / 2),  0,
             0, (delta_t_quadrupled * noise_ay / 4), 0, (delta_t_cubed * noise_ay / 2),
             (delta_t_cubed * noise_ax / 2), 0, (delta_t_squared * noise_ax), 0,
             0, (delta_t_cubed * noise_ay / 2), 0, (delta_t_squared * noise_ay);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // For Radar, the Jacobian matrix is used for H Matrix
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    //For Radar, the measurement noise covariance is size 3x3
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
