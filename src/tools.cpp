#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
VectorXd rmse(4);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
   
    VectorXd diff(4);
	diff << 0,0,0,0;
    VectorXd acc(4);
	acc << 0,0,0,0;
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
		diff = (ground_truth[i]-estimations[i]);
		acc = acc.array() + diff.array()*diff.array();
	}

	//calculate the mean
	// ... your code here
    rmse = acc / estimations.size();
    
	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE
	float sum_squares = px*px+py*py;
	float norm_p=sqrt(sum_squares);
  float H00 = px/norm_p;
  float H01 = py/norm_p;
  float H02 = 0;
  float H03 = 0;
  float H10 = -py/sum_squares;
  float H11 = px/sum_squares;
  float H12 = 0;
  float H13 = 0;
  float H20 = py*(vx*py-vy*px)/sqrt(sum_squares*sum_squares*sum_squares);
  float H21 = px*(vy*px-vx*py)/sqrt(sum_squares*sum_squares*sum_squares);
  float H22 = H00;
  float H23 = H01;
	//check division by zero
	if(fabs(sum_squares) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}
	//compute the Jacobian matrix
    Hj<<H00,H01,H02,H03,H10,H11,H12,H13,H20,H21,H22,H23;
	return Hj;
}
