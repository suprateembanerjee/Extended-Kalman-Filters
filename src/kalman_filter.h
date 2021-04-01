#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter 
{
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman Filter
   * @param x_in Initial State
   * @param P_in Initial State Covariance Matrix
   * @param F_in Transition Matrix
   * @param H_in Measurement Matrix
   * @param R_in Measurement Covariance Matrix
   * @param Q_in Process Covariance Matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // State Vector
  Eigen::VectorXd x_;

  // State Covariance Matrix
  Eigen::MatrixXd P_;

  // State Transition Matrix
  Eigen::MatrixXd F_;

  // Process Covariance Matrix
  Eigen::MatrixXd Q_;

  // Measurement Matrix
  Eigen::MatrixXd H_;

  // Measurement Covariance Matrix
  Eigen::MatrixXd R_;
};

#endif // KALMAN_FILTER_H_
