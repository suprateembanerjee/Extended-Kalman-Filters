#include "kalman_filter.h"
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 * VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
  /**
   * Predict the state
   */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  /**
   * Update the state by using Kalman Filter equations
   */

    MatrixXd y = z - H_ * x_;
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * Update the state by using Extended Kalman Filter equations
   */
  
    float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    float phi = atan2(x_(1), x_(0));
    float rho_dot;
    if (fabs(rho) < 0.0001)
        rho_dot = 0;
    else
        rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;

    MatrixXd y = z - z_pred;

    // Angle Normalization
    if (y(1) > M_PI)
        y(1) -= 2 * M_PI;
    if (y(1) < -M_PI)
        y(1) += 2 * M_PI;

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;

}
