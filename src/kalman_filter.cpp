#include "kalman_filter.h"

#include "tools.h"

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
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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






//cartition -> polar
VectorXd h( VectorXd &x) 
{
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

//#define MIN   0.000001
//  if( x_(0) < MIN )
//        x_(0) = MIN;
//
//  if( x_(1) < MIN )
//        x_(1) = MIN;
//
//

  Tools tools;

    VectorXd v = VectorXd(3);

    int px = x_(0);
    int py = x_(1);
    int vx = x_(2);
    int vy = x_(3);

    v(0) = sqrt(px*px+py*py);

    if( px > -0.0001  ||  px < 0.0001)
       return;

    v(1) = atan2(py,px);

    if( sqrt(px*px+py*py) > -0.0001 || sqrt(px*px+py*py) < 0.0001  )
        return ;

    v(2) = (px*vx + py*vy)/sqrt(px*px+py*py);

    while( v(1) < -3.14 )
    {
        v(1) += 3.14;
    }
    
    while( v(1) > 3.14 )
    {
        v(1) -= 3.14;
    }

  VectorXd z_pred = v;
  VectorXd y = z - z_pred;

 y(1) = atan2(sin(y(1)), cos(y(1)));//nomalize
  while( y(1) < -3.14 )
  {
      printf("YYYYYYYYYYYYYYYYYYYYYYY-   %f\n", y(1));
      y(1) += 3.14;
  }
  
  while( y(1) > 3.14 )
  {
      printf("YYYYYYYYYYYYYYYYYYYYYYY+        %f\n", y(1));
      y(1) -= 3.14;
  }


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
