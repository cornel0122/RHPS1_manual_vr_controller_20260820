#pragma once
#include <Eigen/Dense>

double algebric_derivator(const Eigen::VectorXd & x, const double freq, const int deriv);

Eigen::VectorXd algebric_derivate(const Eigen::VectorXd & X, const int n_window, const double freq, const int deriv);
