#include "kalman_filter.h"

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


#define A_LITTLE_BIT 0.00001
#define PI   3.141592

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
    /*
     * predict the state
     */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
    /*
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
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);

    v(0) = sqrt(px*px+py*py);

    v(1) = A_LITTLE_BIT; //when the denominator px == 0
    if( px != 0 )
        v(1) = atan2(py,px);

    v(2) = A_LITTLE_BIT;
    if((px*px+py*py) != 0) //when the denominator is 0
        v(2) = (px*vx + py*vy)/sqrt(px*px+py*py);

    //normalize the angle. Must be PI <= the angle <= PI
    while( v(1) < -PI ) v(1) += PI;
    while( v(1) > PI ) v(1) -= PI;

    return v;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /*
     * update the state by using Extended Kalman Filter equations
     */

    Tools tools;

    VectorXd z_pred = h(x_);
    VectorXd y = z - z_pred;

    //normalize the angle
    while( y(1) < -PI ) y(1) += PI;
    while( y(1) > PI ) y(1) -= PI;

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
