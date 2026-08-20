#pragma once
#include <Eigen/Dense>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

class RLS_Filter
{
public:
  RLS_Filter() = default;
  RLS_Filter(const int in_shape, const int out_shape)
  {
    input_shape = in_shape;
    output_shape = out_shape;

    Theta = Eigen::VectorXd::Ones(input_shape * output_shape);
    P = Eigen::MatrixXd::Identity(Theta.size(), Theta.size()) * 1e99;
    Rr = Eigen::MatrixXd::Zero(Theta.size(), Theta.size());
  };
  ~RLS_Filter() = default;

  void Estimate(const Eigen::VectorXd & Input);
  void Update_lambda(const double lambda,
                     const Eigen::VectorXd & Input,
                     const Eigen::VectorXd & Real,
                     const Eigen::VectorXd & Pred);
  void Update_alpha(const double alpha,
                    const Eigen::VectorXd & Input,
                    const Eigen::VectorXd & Real,
                    const Eigen::VectorXd & Pred);
  Eigen::VectorXd Estimation;
  Eigen::MatrixXd Estimation_Error;

private:
  double input_shape;
  double output_shape;
  Eigen::VectorXd Theta;

  Eigen::MatrixXd P; // Estimation Error Covariance
  Eigen::MatrixXd K; // Gain Matrix
  Eigen::MatrixXd R; // Estimation Covariance
  Eigen::MatrixXd Rr;

  std::vector<Eigen::VectorXd> theta_error;
  std::vector<Eigen::VectorXd> delta_theta;
};
