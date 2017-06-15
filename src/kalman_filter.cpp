#include "kalman_filter.h"
#include "tools.h"

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
 //u = VectorXd(2);
 //u << 0, 0;
 x_ = F_ * x_; //+ u;
 P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
 // calculate identity matrix
 MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

 // calculate kf parameters
 VectorXd y = z - H_ * x_;
 MatrixXd H_Transpose = H_.transpose();
 MatrixXd S = H_ * P_ * H_Transpose + R_;
 MatrixXd K = (P_ * H_Transpose) * S.inverse();

 // update parameters
 x_ = x_ + (K * y);
 P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations

The main difference between EKF and KF:
- the F matrix will be replaced by F​j​​ when calculating P​′​​.
- the H matrix in the Kalman filter will be replaced by the Jacobian matrix H​j​​ when calculating S, K, and P.
- to calculate x​′​​, the prediction update function, f, is used instead of the F matrix.
- to calculate y, the h function is used instead of the H matrix.

  */

 // calculate identity matrix

 // H_ replaced by Hj in FusionEKF.cpp
 MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

 VectorXd hx = VectorXd(3);
 hx << sqrt(pow(x_(0),2) + pow(x_(1),2)),
		atan2(x_(1),x_(0)),
		(x_(0)*x_(2)+x_(1)*x_(3))/sqrt(pow(x_(0),2) + pow(x_(1),2));
 VectorXd y = z - hx;

 // check this point again
 MatrixXd H_Transpose = H_.transpose();
 MatrixXd S = H_ * P_ * H_Transpose + R_;
 MatrixXd K = (P_ * H_Transpose) * S.inverse();

 // update parameters
 x_ = x_ + (K * y);
 P_ = (I-K*H_)*P_;
}
