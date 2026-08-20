#include <iostream>
#include <stdexcept>

#include "MinJerk_Trajectory.h"

MinJerkTrajectory::MinJerkTrajectory(const double t0_, const double t1_, const double t2_, const double f)
{
  t0 = t0_;
  t1 = t1_;
  t2 = t2_;
  freq = f;

  M_c_psi << 1, pow(t0, 1), pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5), 0, 0, 0, pow(t0, 0), 2 * pow(t0, 1),
      3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4), 0, 0, 0, 0, 2 * pow(t0, 0), 6 * pow(t0, 1), 12 * pow(t0, 2),
      20 * pow(t0, 3), 0, 0, 1, pow(t1, 1), pow(t1, 2), pow(t1, 3), pow(t1, 4), pow(t1, 5), 0, 0, 0, pow(t1, 0),
      2 * pow(t1, 1), 3 * pow(t1, 2), 4 * pow(t1, 3), 5 * pow(t1, 4), 0, 0, 1, pow(t2, 1), pow(t2, 2), pow(t2, 3),
      pow(t2, 4), pow(t2, 5), pow((t2 - t1), 4), pow((t2 - t1), 5), 0, pow(t2, 0), 2 * pow(t2, 1), 3 * pow(t2, 2),
      4 * pow(t2, 3), 5 * pow(t2, 4), 4 * pow((t2 - t1), 3), 5 * pow((t2 - t1), 4), 0, 0, 2 * pow(t2, 0),
      6 * pow(t2, 1), 12 * pow(t2, 2), 20 * pow(t2, 3), 12 * pow((t2 - t1), 2), 20 * pow((t2 - t1), 3);

  M_c_psi_d << 0, pow(t0, 0), 2 * pow(t0, 1), 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4), 0, 0, 0, 0,
      2 * pow(t0, 0), 6 * pow(t0, 1), 12 * pow(t0, 2), 20 * pow(t0, 3), 0, 0, 0, 0, 0, 6 * pow(t0, 0), 24 * pow(t0, 1),
      60 * pow(t0, 2), 0, 0, 0, pow(t1, 0), 2 * pow(t1, 1), 3 * pow(t1, 2), 4 * pow(t1, 3), 5 * pow(t1, 4), 0, 0, 0, 0,
      2 * pow(t1, 0), 6 * pow(t1, 1), 12 * pow(t1, 2), 20 * pow(t1, 3), 0, 0, 0, pow(t2, 0), 2 * pow(t2, 1),
      3 * pow(t2, 2), 4 * pow(t2, 3), 5 * pow(t2, 4), 4 * pow((t2 - t1), 3), 5 * pow((t2 - t1), 4), 0, 0,
      2 * pow(t2, 0), 6 * pow(t2, 1), 12 * pow(t2, 2), 20 * pow(t2, 3), 12 * pow((t2 - t1), 2), 20 * pow((t2 - t1), 3),
      0, 0, 0, 6 * pow(t2, 0), 24 * pow(t2, 1), 60 * pow(t2, 2), 24 * pow((t2 - t1), 2), 60 * pow((t2 - t1), 3);

  M_psi_c = M_c_psi.inverse();
  // M_psi_d_c = M_c_psi.inverse();
}

Eigen::VectorXd MinJerkTrajectory::compute_psi(const Eigen::VectorXd & p01,
                                               const Eigen::VectorXd & a01,
                                               const double delay)
{

  Eigen::QuadProgDense QP;

  const int n01 = p01.size();
  const int window_size = 1;
  const int n_row = a01.size() - (0.5 * ((double)window_size)) + 1;
  const double Te = 1 / freq;
  const Eigen::VectorXd t_window = Eigen::VectorXd::LinSpaced(window_size, 0, ((double)window_size) * Te);
  const double T_w = t_window(window_size - 1);
  const Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_dd = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_acc_algebric = Eigen::MatrixXd::Zero(n_row, 8);
  Eigen::VectorXd b_acc_algebric = Eigen::VectorXd::Zero(n_row);
  Eigen::MatrixXd M_p_algebric = Eigen::MatrixXd::Zero(n_row, 8);
  Eigen::VectorXd b_p_algebric = Eigen::VectorXd::Zero(n_row);
  Eigen::VectorXd b;
  Eigen::VectorXd b_dd;

  for(int i = 0; i < n_row; i++)
  {
    for(int it = 0; it < window_size; it++)
    {

      double t_w = t_window(it);
      double t_row = ((double)i) / freq;
      double alpha_a = (60. / std::pow(T_w, 5)) * (6. * std::pow(t_w, 2) - 6. * T_w * t_w + std::pow(T_w, 2));
      double alpha_p = 2 * ((2 * T_w - 3 * t_w)) / (std::pow(T_w, 2));
      int i_qp = n01 - i + it - ((int)(0.5 * window_size));
      double t_qp = ((double)i_qp) / freq;

      Eigen::VectorXd h = Eigen::VectorXd::Ones(8);
      for(int j = 0; j < 6; j++) { h(j) = std::pow(t_qp, j); }
      for(int j = 6; j < 8; j++)
      {
        int exp = j - 2;
        h(j) = std::pow(std::max(t_qp - t1, 0.), exp);
      }

      M_acc_algebric.row(i) += Te * alpha_a * h;
      M_p_algebric.row(i) += Te * alpha_p * h;
      if(it == 0 || it == window_size - 1)
      {
        M_p_algebric.row(i) -= 0.5 * Te * alpha_p * h;
        M_acc_algebric.row(i) -= 0.5 * Te * alpha_a * h;
      }
    }
    b_acc_algebric(i) = a01(n01 - 1 - i);
    b_p_algebric(i) = p01(n01 - 1 - i);
  }

  for(int i = 0; i < n01; i++)
  {
    for(int j = 0; j < 6; j++)
    {

      M(i, j) = pow(t_01(i), j);
      if(j >= 2) { M_dd(i, j) = j * (j - 1) * pow(t_01(i), j - 2); }
    }
  }

  b_dd = a01;
  b = p01;

  Eigen::MatrixXd H = M * M_psi_c;
  Eigen::MatrixXd H_dd = M_dd * M_psi_c;
  Eigen::MatrixXd H_acc_algebric = M_acc_algebric * M_psi_c;
  Eigen::MatrixXd H_p_algebric = M_p_algebric * M_psi_c;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(8, 8) * 1e-12 + (H.transpose() * H) + (H_dd.transpose() * H_dd)
                      + 0 * (H_acc_algebric.transpose() * H_acc_algebric)
                      + 0 * (H_p_algebric.transpose() * H_p_algebric);

  Eigen::VectorXd p = (-H.transpose() * b) + (-H_dd.transpose() * b_dd)
                      + 0 * (-H_acc_algebric.transpose() * b_acc_algebric)
                      + 0 * (-H_p_algebric.transpose() * b_p_algebric);

  int Nvar = Q.rows();
  int NIneqConstr = 1;
  int NEqConstr = 1;
  // QP.tolerance(1e-4);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  bool success = QP.solve(Q, p, Eigen::MatrixXd::Zero(1, 8), Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, 8),
                          Eigen::VectorXd::Zero(1));

  Eigen::VectorXd QP_out = QP.result();
  psi = QP_out;
  // std::cout << "a01" << std::endl << a01 << std::endl << "b01" << std::endl << p01 << std::endl;
  set_C_from_psi(psi);

  return psi;
}

Eigen::VectorXd MinJerkTrajectory::compute_psi(const Eigen::VectorXd & p01, const Eigen::VectorXd & v01)
{

  Eigen::QuadProgDense QP;

  const int n01 = p01.size();
  const double Te = 1 / freq;
  const Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_d = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::VectorXd b;
  Eigen::VectorXd b_d;

  for(int i = 0; i < n01; i++)
  {
    for(int j = 0; j < 6; j++)
    {

      M(i, j) = pow(t_01(i), j);
      if(j >= 1) { M_d(i, j) = j * pow(t_01(i), j - 1); }
    }
  }

  b = p01;
  b_d = v01;

  Eigen::MatrixXd H = M * M_psi_c;
  Eigen::MatrixXd H_d = M_d * M_psi_c;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(8, 8) * 1e-12 + (H.transpose() * H) + 1. * (H_d.transpose() * H_d);

  Eigen::VectorXd p = (-H.transpose() * b) + 1. * (-H_d.transpose() * b_d);

  int Nvar = Q.rows();
  int NIneqConstr = 1;
  int NEqConstr = 1;
  // QP.tolerance(1e-4);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  bool success = QP.solve(Q, p, Eigen::MatrixXd::Zero(1, 8), Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, 8),
                          Eigen::VectorXd::Zero(1));

  Eigen::VectorXd QP_out = QP.result();
  psi = QP_out;
  // std::cout << "a01" << std::endl << a01 << std::endl << "b01" << std::endl << p01 << std::endl;
  set_C_from_psi(psi);

  return psi;
}

Eigen::VectorXd MinJerkTrajectory::compute_psi(const Eigen::VectorXd & p01,
                                               const Eigen::VectorXd & v_01,
                                               const Eigen::VectorXd & a01,
                                               const Eigen::VectorXd & j01,
                                               const double delay)
{

  Eigen::QuadProgDense QP;
  const int n01 = p01.size();
  const int window_size = 30;
  const int n_row = a01.size() - (0.5 * ((double)window_size)) + 1;
  const double Te = 1 / freq;
  const Eigen::VectorXd t_window = Eigen::VectorXd::LinSpaced(window_size, 0, ((double)window_size) * Te);
  const double T_w = t_window(window_size - 1);
  const Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_d = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_dd = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_j = Eigen::MatrixXd::Zero(n01, 8);
  Eigen::MatrixXd M_acc_algebric = Eigen::MatrixXd::Zero(n_row, 8);
  Eigen::VectorXd b_acc_algebric = Eigen::VectorXd::Zero(n_row);
  Eigen::VectorXd b;
  Eigen::VectorXd b_d;
  Eigen::VectorXd b_dd;
  Eigen::VectorXd b_j;

  for(int i = 0; i < n_row; i++)
  {

    for(int it = 0; it < window_size; it++)
    {

      double t_w = t_window(it);
      double t_row = ((double)i) / freq;
      double alpha =
          (120. / (2. * (std::pow(T_w, 5)))) * (std::pow(T_w - t_w, 2) - t_w * (4 * (T_w - t_w)) + std::pow(t_w, 2));
      double t_qp = t1 - t_row + t_w - 0.5 * T_w;
      Eigen::VectorXd h = Eigen::VectorXd::Ones(8);
      for(int j = 0; j < 6; j++) { h(j) = std::pow(t_qp, j); }
      for(int j = 6; j < 8; j++)
      {
        int exp = j - 2;
        h(j) = std::pow(std::max(t_qp - t1, 0.), exp);
      }
      M_acc_algebric.row(i) += Te * alpha * h;
      if(it == 0 || it == window_size - 1) { M_acc_algebric.row(i) -= 0.5 * Te * alpha * h; }

      b_acc_algebric(i) = a01(n01 - 1 - i);
    }
  }

  for(int i = 0; i < n01; i++)
  {
    for(int j = 0; j < 6; j++)
    {

      M(i, j) = pow(t_01(i), j);
      if(j >= 1) { M_d(i, j) = j * pow(t_01(i) - delay, j - 1); }
      if(j >= 2) { M_dd(i, j) = j * (j - 1) * pow(t_01(i), j - 2); }
      if(j >= 3) { M_j(i, j) = j * (j - 1) * (j - 2) * pow(t_01(i) - delay, j - 3); }
    }
  }

  b_dd = a01;
  b = p01;
  b_d = v_01;
  b_j = j01;

  Eigen::MatrixXd H = M * M_psi_c;
  Eigen::MatrixXd H_d = M_d * M_psi_c;
  Eigen::MatrixXd H_dd = M_dd * M_psi_c;
  Eigen::MatrixXd H_j = M_j * M_psi_c;
  Eigen::MatrixXd H_acc_algebric = M_acc_algebric * M_psi_c;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(8, 8) * 1e-12 + (H.transpose() * H) + (H_dd.transpose() * H_dd)
                      + (H_j.transpose() * H_j)
                      + (H_acc_algebric.transpose() * H_acc_algebric); //+  (H_d.transpose() * H_d);
  Eigen::VectorXd p = (-H.transpose() * b) + (-H_dd.transpose() * b_dd) + (-H_j.transpose() * b_j)
                      + (-H_acc_algebric.transpose() * b_acc_algebric); //+ (-H_d.transpose() * b_d)

  int Nvar = Q.rows();
  int NIneqConstr = 1;
  int NEqConstr = 1;
  // QP.tolerance(1e-4);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  bool success = QP.solve(Q, p, Eigen::MatrixXd::Zero(1, 8), Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, 8),
                          Eigen::VectorXd::Zero(1));

  Eigen::VectorXd QP_out = QP.result();
  psi = QP_out;
  // std::cout << "a01" << std::endl << a01 << std::endl << "b01" << std::endl << p01 << std::endl;
  set_C_from_psi(psi);

  return psi;
}

Eigen::VectorXd MinJerkTrajectory::compute_true_psi(const Eigen::VectorXd & p01, const Eigen::VectorXd & p12)
{

  Eigen::QuadProgDense QP;
  double delta = t2 - t1;
  int n12 = p12.size();
  int n01 = p01.size();
  int n02 = n12 + n01;
  double f = freq;

  Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(p01.size(), 0, t1);
  Eigen::VectorXd t_12 = Eigen::VectorXd::LinSpaced(p12.size(), t1 + 1 / f, t2);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n02, 8);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n02);

  for(int i = 0; i < n01; i++)
  {
    for(int j = 0; j < 6; j++) { M(i, j) = pow(t_01(i), j); }

    b(i) = p01(i);
  }
  for(int i = 0; i < n12; i++)
  {
    for(int j = 0; j < 6; j++) { M(i + n01, j) = pow(t_12(i), j); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(i + n01, j) = pow(t_12(i) - t1, exp);
    }

    b(i + n01) = p12(i);
  }

  Eigen::MatrixXd H = M * M_psi_c;

  Eigen::MatrixXd Q = H.transpose() * H + Eigen::MatrixXd::Identity(8, 8) * 1e-12;

  Eigen::VectorXd p = (-H.transpose() * b);

  int Nvar = Q.rows();
  int NIneqConstr = 1;
  int NEqConstr = 1;
  // // QP.tolerance(1e-4);
  QP.problem(Nvar, NEqConstr, NIneqConstr);
  bool success = QP.solve(Q, p, Eigen::MatrixXd::Zero(1, 8), Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Zero(1, 8),
                          Eigen::VectorXd::Zero(1));

  Eigen::VectorXd QP_out = QP.result();
  psi = QP_out;
  set_C_from_psi(psi);

  return psi;
}

Eigen::VectorXd MinJerkTrajectory::compute_position(const double n01, const double n12)
{

  double delta = t2 - t1;
  double f = freq;

  Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);
  Eigen::VectorXd t_12 = Eigen::VectorXd::LinSpaced(n12, t1 + 1 / f, t2);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(t_01.size() + t_12.size(), 8);

  for(int i = 0; i < t_01.size(); i++)
  {
    for(int j = 0; j < 6; j++) { M(i, j) = pow(t_01(i), j); }
  }
  for(int i = 0; i < t_12.size(); i++)
  {
    for(int j = 0; j < 6; j++) { M(i + t_01.size(), j) = pow(t_12(i), j); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(i + t_01.size(), j) = pow(t_12(i) - t1, exp);
    }
  }

  return M * c;
}

Eigen::VectorXd MinJerkTrajectory::compute_velocity(const double n01, const double n12)
{

  double delta = t2 - t1;
  double f = freq;

  Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);
  Eigen::VectorXd t_12 = Eigen::VectorXd::LinSpaced(n12, t1 + 1 / f, t2);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(t_01.size() + t_12.size(), 8);

  for(int i = 0; i < t_01.size(); i++)
  {
    for(int j = 1; j < 6; j++) { M(i, j) = j * pow(t_01(i), j - 1); }
  }
  for(int i = 0; i < t_12.size(); i++)
  {
    for(int j = 1; j < 6; j++) { M(i + t_01.size(), j) = j * pow(t_12(i), j - 1); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(i + t_01.size(), j) = (exp - 1) * pow(t_12(i) - t1, exp - 1);
    }
  }

  return M * c;
}

Eigen::VectorXd MinJerkTrajectory::compute_acceleration(const double n01, const double n12)
{

  double delta = t2 - t1;
  double f = freq;

  Eigen::VectorXd t_01 = Eigen::VectorXd::LinSpaced(n01, 0, t1);
  Eigen::VectorXd t_12 = Eigen::VectorXd::LinSpaced(n12, t1 + 1 / f, t2);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(t_01.size() + t_12.size(), 8);

  for(int i = 0; i < t_01.size(); i++)
  {
    for(int j = 2; j < 6; j++) { M(i, j) = j * (j - 1) * pow(t_01(i), j - 2); }
  }

  for(int i = 0; i < t_12.size(); i++)
  {
    for(int j = 2; j < 6; j++) { M(i + t_01.size(), j) = j * (j - 1) * pow(t_12(i), j - 2); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(i + t_01.size(), j) = exp * (exp - 1) * pow(t_12(i) - t1, exp - 2);
    }
  }

  return M * c;
}

double MinJerkTrajectory::get_pose(double t)
{
  Eigen::VectorXd M = Eigen::VectorXd::Zero(8);

  if(t < t1)
  {
    for(int j = 0; j < 6; j++) { M(j) = pow(t, j); }
  }
  else
  {

    for(int j = 0; j < 6; j++) { M(j) = pow(t, j); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(j) = pow(t - t1, exp);
    }
  }

  return static_cast<double>(M.transpose() * c);
}

double MinJerkTrajectory::get_vel(double t)
{

  Eigen::VectorXd M = Eigen::VectorXd::Zero(8);

  if(t < t1)
  {
    for(int j = 1; j < 6; j++) { M(j) = j * pow(t, j - 1); }
  }
  else
  {
    for(int j = 1; j < 6; j++) { M(j) = j * pow(t, j - 1); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(j) = (exp - 1) * pow(t - t1, exp - 1);
    }
  }

  return static_cast<double>(M.transpose() * c);
  ;
}
double MinJerkTrajectory::get_acc(double t)
{

  Eigen::VectorXd M = Eigen::VectorXd::Zero(8);

  if(t < t1)
  {
    for(int j = 2; j < 6; j++) { M(0, j) = j * (j - 1) * pow(t, j - 2); }
  }

  else
  {
    for(int j = 2; j < 6; j++) { M(0, j) = j * (j - 1) * pow(t, j - 2); }
    for(int j = 6; j < 8; j++)
    {
      int exp = j - 2;
      M(0, j) = exp * (exp - 1) * pow(t, exp - 2);
    }
  }

  return static_cast<double>(M.transpose() * c);
  ;
}
