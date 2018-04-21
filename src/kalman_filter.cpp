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
    VectorXd v = VectorXd(3);

    int px = x(0);
    int py = x(1);
    int vx = x(2);
    int vy = x(3);

    v(0) = sqrt(px*px+py*py);

    if( px != 0 )
        v(1) = atan2(py,px);

    if( (px*px+py*py) != 0 )
        v(2) = (px*vx + py*vy)/sqrt(px*px+py*py);

while( v(1) < -3.14 )
{
    v(1) += 3.14;
}

while( v(1) > 3.14 )
{
    v(1) -= 3.14;
}

    return v;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(x_);

  //VectorXd z_pred = Hj*x_;
  VectorXd z_pred = h(x_);
  VectorXd y = z - z_pred;
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


  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHjt = P_ * Hjt;
  MatrixXd K = PHjt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;


}
