#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.F_ = MatrixXd(4, 4);

  // Measurement Covariance Matrix - Laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement Covariance Matrix - RaDAR
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */

  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Convert radar from polar to cartesian coordinates.
     */

    // First measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
        // Convert radar from polar to cartesian coordinates and initialize state.

        float rho = measurement_pack.raw_measurements_(0);
        float phi = measurement_pack.raw_measurements_(1);
        float rho_dot = measurement_pack.raw_measurements_(2);
        
        float x = rho * cos(phi);

        if (x < 0.0001)
            x = 0.0001;

        float y = rho * sin(phi);

        if (y < 0.0001)
            y = 0.0001;

        double vx = rho_dot * cos(phi);
        double vy = rho_dot * sin(phi);

        ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      // Initialize state

        ekf_.x_(0) = measurement_pack.raw_measurements_(0);
        ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // Initializing completed, no need to predict or update

    is_initialized_ = true;
    return;
  }

  /*************
   * Prediction
   *************/

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double elapsed_time = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  VectorXd dt = VectorXd(5);
  dt << 1, elapsed_time, pow(elapsed_time, 2), pow(elapsed_time, 3), pow(elapsed_time, 4);
  ekf_.F_(0,2) = ekf_.F_(1, 3) = dt(1);
  
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt(4) * noise_ax / 4, 0, dt(3)* noise_ax / 2, 0,
             0, dt(4)* noise_ay / 4, 0, dt(3)* noise_ay / 2,
             dt(3)* noise_ax / 2, 0, dt(2)* noise_ax, 0,
             0, dt(3)* noise_ay / 2, 0, dt(2)* noise_ay;

  ekf_.Predict();

  /*********
   * Update
   *********/

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
  {
      // Radar updates
      Hj_ = Tools().CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else 
  {
      // Laser updates
      ekf_.R_ = R_laser_;
      ekf_.H_ = H_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);

  }

  // Update the previous timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // Print the Output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}