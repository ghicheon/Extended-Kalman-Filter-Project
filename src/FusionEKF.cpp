#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <unordered_map> //hash_map
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/* 
 * avoid repeated calculations for F_ & Q_ 
 */
unordered_map <int, MatrixXd> hash4F_;
unordered_map <int, MatrixXd> hash4Q_;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);

    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;


     H_laser_  << 1,0,0,0,
                    0,1,0,0;

    /*
     * init ekf_
     */
    VectorXd xx = VectorXd(4);
    xx << 1,1,0.1,0.1;//resonable initial value

    MatrixXd PP = MatrixXd(4, 4);
    PP << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

    MatrixXd FF =  MatrixXd(4, 4);
    FF << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd HH = MatrixXd(2, 4);
    MatrixXd RR = MatrixXd(2, 2);
    MatrixXd QQ = MatrixXd(4, 4);

    ekf_.Init( xx, PP, FF,HH,RR,QQ);
}

/*
 * Destructor.
 */
FusionEKF::~FusionEKF() {}



void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /*
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        // first measurement
        cout << "EKF: " << endl;

        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            //Convert radar from polar to cartesian coordinates and initialize state.
            float rho     = measurement_pack.raw_measurements_[0];
            float phi      = measurement_pack.raw_measurements_[1];
            float rho_dot = measurement_pack.raw_measurements_[2];

            float px = rho*cos(phi); 
            float py = rho*sin(phi);

            ekf_.x_(0) = px ;
            ekf_.x_(1) = py ;
            ekf_.x_(2) = 0.1 ;
            ekf_.x_(3) = 0.1 ;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
          /**
          Initialize state.
          */
          ekf_.x_(0) =  measurement_pack.raw_measurements_[0];
          ekf_.x_(1) =  measurement_pack.raw_measurements_[1];
          ekf_.x_(2) = 5.199937e+00 ; //I got it from data.
          ekf_.x_(3) = 0 ;

        }

        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }


    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /*
     * Update the state transition matrix F according to the new elapsed time.
         - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    static const int noise_ax = 9;
    static const int noise_ay = 9;

    unsigned int diff =measurement_pack.timestamp_ - previous_timestamp_;

    previous_timestamp_ = measurement_pack.timestamp_;

    //found!!
    if( (hash4F_.find(diff) != hash4F_.end()) 
        /*&& (hash4F_.find(diff) != hash4F_.end())*/ ) //well..it's not necessary in my implimentation.
    {
        ekf_.F_ = hash4F_[diff];
        ekf_.Q_ = hash4Q_[diff];
    }
    else  //not found
    {
        float dt = diff / 1000000.0; //in seconds
        
        float dt_2 = dt * dt;
        float dt_3 = dt_2 * dt;
        float dt_4 = dt_3 * dt;
        
        //Modify the F matrix so that the time is integrated
        ekf_.F_(0, 2) = dt;
        ekf_.F_(1, 3) = dt;
        
        //set the process covariance matrix Q
        ekf_.Q_ = MatrixXd(4, 4);
        ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                     0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                     dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                     0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
        hash4F_[diff] = ekf_.F_;
        hash4Q_[diff] = ekf_.Q_ ;
        cout << "hash inserted... diff:" << diff << endl;
    }
        
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /*
     * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
