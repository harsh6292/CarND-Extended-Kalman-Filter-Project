#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;

  // If estimations size is not equal to ground truth, cannot calculate rmse
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
  	cout<<"Invalid estimation or ground truth data size"<<endl;
  	return rmse;
  }

  for(unsigned int i = 0; i < estimations.size(); ++i) {
  	// Get the residual (Error in RMSE)
  	VectorXd residual = (estimations[i] - ground_truth[i]);

  	// Get the square of residual (Squared in RMSE)
  	residual = (residual.array() * residual.array());

  	//Add the residual to overall rmse (Add the squared errors)
  	rmse += residual;
  }

  // Get the average of all rmse values (Mean in RMSE)
  rmse = rmse/estimations.size();

  //Calculate the square root of summed rmse (Root mean squared error)
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  Hj << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  // Store the current object position
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = ( (px * px) + (py * py) );
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
