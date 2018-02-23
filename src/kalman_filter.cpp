#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  // Predict new location of object using state transition matrix
  x_ = (F_ * x_);

  // Predict the new state covariance matrix
  Eigen::MatrixXd F_transpose_ = F_.transpose();
  P_ = (F_ * P_ * F_transpose_) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // Laser Update Kalman Filter equation

  // Equation-1
  VectorXd y_ = ( z - (H_ * x_) );

  MatrixXd H_transpose_ = H_.transpose();

  // Equation-2
  MatrixXd S_ = ( (H_ * P_ * H_transpose_) + R_ );

  MatrixXd S_inverse_ = S_.inverse();

  // Equation-3
  MatrixXd K_ = (P_ * H_transpose_ * S_inverse_);

  // Equation-4 : Update the location of object using laser measurements
  x_ = ( x_ + (K_ * y_) );

  // Equation-5 : Update the state covariance matrix using laser measurements
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);

  P_ = ( (I_ - (K_ * H_)) * P_ );
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Radar Update Kalman Filter equation

  // First convert predicted cartesian coordinates into polar coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float px_2 = (px * px);
  float py_2 = (py * py);

  float rho = sqrt((px_2 + py_2));
  float phi = atan2(py/px);
  float rho_dot = (( (px * vx) + (py * vy) ) / rho);

  // Store the predicted converted values in a vector
  Eigen:VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;


  // Equation-1
  VectorXd y_ = ( z - z_pred );


  // Let's normalize the angle phi to be between -pi and +pi
  
  MatrixXd H_transpose_ = H_.transpose();

  // Equation-2
  MatrixXd S_ = ( (H_ * P_ * H_transpose_) + R_ );

  MatrixXd S_inverse_ = S_.inverse();

  // Equation-3
  MatrixXd K_ = (P_ * H_transpose_ * S_inverse_);

  // Equation-4 : Update the location of object using laser measurements
  x_ = ( x_ + (K_ * y_) );

  // Equation-5 : Update the state covariance matrix using laser measurements
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size);

  P_ = ( (I_ - (K_ * H_)) * P_ );
}
