#pragma once
#include "Kalman_Filter.h"
#include "MinJerk_Trajectory.h"
#include "RLS_Filter.h"
#include <Eigen/Dense>
#include <Eigen/QR>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

class Motion_Prediction
{

public:
  Motion_Prediction() = default;
  Motion_Prediction(const int freq, const int N1, const int N2, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);
  ~Motion_Prediction() = default;

  void Compute_Prediction_Trajectory_(const Eigen::MatrixXd & Traj, const Eigen::MatrixXd & Acc, const double delay);
  void Compute_Prediction_Orientaion_Trajectory(const Eigen::MatrixXd & Ori, const Eigen::MatrixXd & Gyro);

  Eigen::Vector3d get_prediction_Pos_coordinate(const double t);
  Eigen::Vector3d get_prediction_Vel_coordinate(const double t);
  Eigen::Vector3d get_prediction_Acc_coordinate(const double t);

  bool Use_Kalman_Filter = false;

  double rls_lambda() { return lambda; }
  void rls_lambda(const double l) { lambda = l; }
  double rls_alpha() { return alpha; }
  void rls_alpha(const double a) { alpha = a; }
  int frequency() { return frequency_; }

private:
  double MAX_DIST = 0.15;
  int frequency_ = 60;
  int N_1 = 5;
  int N_2 = 5;
  double t0, t1, t2;
  double lambda = 1.005;
  double alpha = 0.1;
  double Layer_Switch_thresh = 50;

  Eigen::Matrix<double, 3, 8> coeffs_;

  MinJerkTrajectory TrajMinJerk;

  RLS_Filter Prediction_Layer_alpha;

  RLS_Filter Prediction_Layer_lambda;

  KalmanFilter F_x;
  KalmanFilter F_y;
  KalmanFilter F_z;
};
