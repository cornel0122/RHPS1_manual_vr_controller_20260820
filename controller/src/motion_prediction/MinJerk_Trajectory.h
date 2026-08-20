#include "eigen-quadprog/QuadProg.h"
#include "eigen-quadprog/eigen_quadprog_api.h"
#include <Eigen/Dense>
#pragma once

class MinJerkTrajectory
{

public:
  MinJerkTrajectory(const double t0_, const double t1_, const double t2_, const double f);
  MinJerkTrajectory(){};

  Eigen::VectorXd compute_psi(const Eigen::VectorXd & p01, const Eigen::VectorXd & a01, const double delay);
  Eigen::VectorXd compute_psi(const Eigen::VectorXd & p01, const Eigen::VectorXd & v01);

  Eigen::VectorXd compute_psi(const Eigen::VectorXd & p01,
                              const Eigen::VectorXd & v01,
                              const Eigen::VectorXd & a01,
                              const Eigen::VectorXd & j01,
                              const double delay);

  Eigen::VectorXd compute_true_psi(const Eigen::VectorXd & p01, const Eigen::VectorXd & p12);
  Eigen::VectorXd compute_position(const double n01, const double n12);
  Eigen::VectorXd compute_velocity(const double n01, const double n12);
  Eigen::VectorXd compute_acceleration(const double n01, const double n12);

  double get_pose(double t);

  double get_vel(double t);

  double get_acc(double t);

  Eigen::VectorXd get_psi() { return psi; };

  Eigen::VectorXd get_c() { return c; }
  void set_c(const Eigen::VectorXd & coeffs) { c = coeffs; }

  Eigen::MatrixXd get_M_psi_c() { return M_psi_c; }
  Eigen::MatrixXd get_M_c_psi() { return M_c_psi; }

  void set_C_from_psi(const Eigen::VectorXd & psi)
  {
    c = M_psi_c * psi;
    set_psi(psi);
  };

  void set_psi(const Eigen::VectorXd & psi_) { psi = psi_; }

private:
  double t0, t1, t2;
  double freq = 60;

  Eigen::VectorXd c = Eigen::VectorXd::Zero(8);
  Eigen::VectorXd psi = Eigen::VectorXd::Zero(8);
  Eigen::MatrixXd M_psi_c = Eigen::MatrixXd::Zero(8, 8);
  Eigen::MatrixXd M_c_psi = Eigen::MatrixXd::Zero(8, 8);
  Eigen::MatrixXd M_c_psi_d = Eigen::MatrixXd::Zero(8, 8);
};
