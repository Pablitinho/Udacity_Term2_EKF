#include "kalman_filter.h"
#include <iostream>
#include "math.h"
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_laser, MatrixXd &R_laser,
                        MatrixXd &R_radar, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_laser;
  R_laser_ = R_laser;
  R_radar_ = R_radar;
  Q_ = Q_in;
  I = MatrixXd::Identity(4, 4);
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
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_laser_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
  /* Kalman Gain*/
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	//long x_size = x_.size();
	//MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  /* Transform from Cartesian to Polar Coordinate system */
  VectorXd h_x(3);
  h_x(0) = sqrt(px * px + py * py);
  h_x(1) = atan2(py,px);
  h_x(2) = (px*vx+py*vy)/ (h_x(0)+0.0001);

  /* Jacobian matrix to linealized the function*/
  MatrixXd H_radar = this->my_tools.CalculateJacobian(x_);

  VectorXd y = z - h_x;
  /* Ensure that the range is from -PI to PI*/
  if( y[1] > M_PI )
  {
      y[1] -= 2.f*M_PI;
  }
    
  if( y[1] < -M_PI )
  {
     y[1] += 2.f*M_PI;
  }
    
  /* Same like the Lidar measurement but using the Jacobian matrix*/
  MatrixXd Ht = H_radar.transpose();

	MatrixXd S = H_radar * P_ * Ht + R_radar_;

  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;

  MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	//MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_radar) * P_;
}
