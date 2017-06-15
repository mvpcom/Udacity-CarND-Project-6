#include <iostream>
#include "tools.h"

using namespace std;
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
 VectorXd rmse(4);
 rmse << 0,0,0,0;

 // check the validity of inputs
 if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
   cout << "Not Validation Error!!!";
   return rmse; 
}
 
 // accumulate squared residuals
 for (int i=0; i < estimations.size(); ++i){
   VectorXd residual = estimations[i]-ground_truth[i];
   residual = residual.array() * residual.array();
   rmse += residual;  
 }

 rmse /= estimations.size();
 rmse = rmse.array().sqrt();

 return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
 MatrixXd Hj(3,4);
 //recover sate parameters
 float px = x_state(0);
 float py = x_state(1);
 float vx = x_state(2);
 float vy = x_state(3);

 //check devision by zero
 if (px == 0 || py == 0){
   cout << "Oaps, Division by zero! :(";
 }
 
 //compute the Jacobian Matrix
 float denum = pow(px,2) + pow(py,2);
 float denumSqrt = sqrt(denum);
 Hj << px/denumSqrt, py/denumSqrt, 0, 0,
      -py/denum, px/denum, 0, 0,
       py*(vx*py-vy*px)/(denum*denumSqrt), px*(vy*px-vx*py)/(denum*denumSqrt), px/denumSqrt, py/denumSqrt;
 return Hj;
}
