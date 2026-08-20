#include "Motion_Prediction.h"

Motion_Prediction::Motion_Prediction(const int freq,
                                     const int N1,
                                     const int N2,
                                     const Eigen::MatrixXd & Q,
                                     const Eigen::MatrixXd & R)
{
  frequency_ = freq;
  N_1 = N1;
  N_2 = N2;
  double dt = 1 / static_cast<double>(frequency_);
  Prediction_Layer_alpha = RLS_Filter(8, 8);
  Prediction_Layer_lambda = RLS_Filter(8, 8);

  t0 = 0;
  t1 = (static_cast<double>(N1) - 1.) * dt;
  t2 = (static_cast<double>(N1 + N2) - 1.) * dt;
  TrajMinJerk = MinJerkTrajectory(0, t1, t2, static_cast<double>(frequency_));

  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(8, 8);
  Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(8, 8);
  double t0p = t0 + dt;
  double t1p = t1 + dt;
  double t2p = t2 + dt;
  A_c << 1, pow(t0p, 1), pow(t0p, 2), pow(t0p, 3), pow(t0p, 4), pow(t0p, 5), 0, 0, 0, pow(t0p, 0), 2 * pow(t0p, 1),
      3 * pow(t0p, 2), 4 * pow(t0p, 3), 5 * pow(t0p, 4), 0, 0, 0, 0, 2 * pow(t0p, 0), 6 * pow(t0p, 1), 12 * pow(t0p, 2),
      20 * pow(t0p, 3), 0, 0, 1, pow(t1p, 1), pow(t1p, 2), pow(t1p, 3), pow(t1p, 4), pow(t1p, 5), pow((t1p - t1), 4),
      pow((t1p - t1), 5), 0, pow(t1p, 0), 2 * pow(t1p, 1), 3 * pow(t1p, 2), 4 * pow(t1p, 3), 5 * pow(t1p, 4),
      4 * pow((t1p - t1), 3), 5 * pow((t1p - t1), 4), 1, pow(t2p, 1), pow(t2p, 2), pow(t2p, 3), pow(t2p, 4),
      pow(t2p, 5), pow((t2p - t1), 4), pow((t2p - t1), 5), 0, pow(t2p, 0), 2 * pow(t2p, 1), 3 * pow(t2p, 2),
      4 * pow(t2p, 3), 5 * pow(t2p, 4), 4 * pow((t2p - t1), 3), 5 * pow((t2p - t1), 4), 0, 0, 2 * pow(t2p, 0),
      6 * pow(t2p, 1), 12 * pow(t2p, 2), 20 * pow(t2p, 3), 12 * pow((t2p - t1), 2), 20 * pow((t2p - t1), 3);

  Eigen::MatrixXd A = A_c * TrajMinJerk.get_M_psi_c();
  F_x = KalmanFilter(dt, A, C, Q, R, Eigen::MatrixXd::Identity(8, 8));
  F_y = KalmanFilter(dt, A, C, Q, R, Eigen::MatrixXd::Identity(8, 8));
  F_z = KalmanFilter(dt, A, C, Q, R, Eigen::MatrixXd::Identity(8, 8));
  F_x.init();
  F_y.init();
  F_z.init();

  coeffs_.setZero();

  std::cout << "motion prediction initialized at frequence " << frequency_ << " with : " << std::endl
            << "N1 : " << N_1 << " t1 : " << t1 << std::endl
            << "N2 : " << N2 << " t2 : " << t2 << std::endl;
};

void Motion_Prediction::Compute_Prediction_Orientaion_Trajectory(const Eigen::MatrixXd & Ori,
                                                                 const Eigen::MatrixXd & Gyro)

{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  int N_in = Ori.cols();

  Eigen::VectorXd T(Eigen::VectorXd::LinSpaced(N_in, t0, t2));
  Eigen::VectorXd T_cut(Eigen::VectorXd::LinSpaced(N_1, t0, t1));
  Eigen::VectorXd T_pred(Eigen::VectorXd::LinSpaced(N_2, t1, t2));

  for(int axis = 0; axis < 3; axis++)
  {
    // Training

    Eigen::VectorXd ori_ax = Ori.row(axis); // Trajectory in one axis
    Eigen::VectorXd gyro_ax = Gyro.row(axis); // Acceleration in one axis
    Eigen::VectorXd ori_ax_cutted = ori_ax.segment(N_in - N_2 - N_1, N_1);
    Eigen::VectorXd gyro_ax_cutted = gyro_ax.segment(N_in - N_2 - N_1, N_1);

    Eigen::VectorXd ori_ax_pred = ori_ax.segment(N_in - N_2, N_2);

    Eigen::VectorXd psi = TrajMinJerk.compute_true_psi(ori_ax_cutted, ori_ax_pred);
    Eigen::VectorXd psi_measure = TrajMinJerk.compute_psi(ori_ax_cutted, gyro_ax_cutted);

    Prediction_Layer_lambda.Estimate(psi_measure);
    Prediction_Layer_lambda.Update_lambda(lambda, psi_measure, psi, Prediction_Layer_lambda.Estimation);

    // Prediction_Layer_alpha.Estimate(psi_measure);
    // Prediction_Layer_alpha.Update_alpha(alpha,psi_measure,psi,Prediction_Layer_alpha.Estimation);

    // Estimation

    ori_ax_cutted = ori_ax.segment(N_in - N_1, N_1);
    gyro_ax_cutted = gyro_ax.segment(N_in - N_1, N_1);

    psi_measure = TrajMinJerk.compute_psi(ori_ax_cutted, gyro_ax_cutted);
    // Prediction_Layer_alpha.Estimate(psi_measure);
    Prediction_Layer_lambda.Estimate(psi_measure);

    Eigen::VectorXd psi_rls_corr = Prediction_Layer_lambda.Estimation;

    // Eigen::VectorXd psi_filtered = psi_rls_corr;
    Eigen::VectorXd psi_filtered = psi_measure;

    // if(Use_Kalman_Filter)
    // {
    //   if(axis == 0)
    //   {
    //     F_x.update(psi_rls_corr);
    //     psi_filtered = F_x.state();
    //   }
    //   if(axis == 1)
    //   {
    //     F_y.update(psi_rls_corr);
    //     psi_filtered = F_y.state();
    //   }
    //   if(axis == 2)
    //   {
    //     F_z.update(psi_rls_corr);
    //     psi_filtered = F_z.state();
    //   }
    // }

    // if(psi_filtered(5) - ori_ax(N_in - 1) > MAX_DIST)
    // {

    //   psi_filtered(7) = 0;
    //   psi_filtered(6) = 0;
    //   psi_filtered(5) = ori_ax(N_in - 1) + MAX_DIST;
    // }
    // if(psi_filtered(5) - ori_ax(N_in - 1) < -MAX_DIST)
    // {

    //   psi_filtered(7) = 0;
    //   psi_filtered(6) = 0;
    //   psi_filtered(5) = ori_ax(N_in - 1) - MAX_DIST;
    // }

    TrajMinJerk.set_C_from_psi(psi_filtered);
    coeffs_.row(axis) = TrajMinJerk.get_c();
  }
  // std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // std::cout << "elapsed time " << time_span.count() << std::endl;
}

void Motion_Prediction::Compute_Prediction_Trajectory_(const Eigen::MatrixXd & Traj,
                                                       const Eigen::MatrixXd & Acc,
                                                       const double delay)
{
  std::chrono::high_resolution_clock::time_point t_clock = std::chrono::high_resolution_clock::now();

  int N_in = Traj.cols();

  Eigen::VectorXd T(Eigen::VectorXd::LinSpaced(N_in, t0, t2));
  Eigen::VectorXd T_cut(Eigen::VectorXd::LinSpaced(N_1, t0, t1));
  Eigen::VectorXd T_pred(Eigen::VectorXd::LinSpaced(N_2, t1, t2));

  for(int axis = 0; axis < 3; axis++)
  {
    // Training

    Eigen::VectorXd traj_ax = Traj.row(axis); // Trajectory in one axis
    Eigen::VectorXd acc_ax = Acc.row(axis); // Acceleration in one axis
    Eigen::VectorXd traj_ax_cutted = traj_ax.segment(N_in - N_2 - N_1, N_1);
    Eigen::VectorXd acc_ax_cutted = acc_ax.segment(N_in - N_2 - N_1, N_1);
    ;

    Eigen::VectorXd traj_ax_pred = traj_ax.segment(N_in - N_2, N_2);

    Eigen::VectorXd psi = TrajMinJerk.compute_true_psi(traj_ax_cutted, traj_ax_pred);
    Eigen::VectorXd psi_measure = TrajMinJerk.compute_psi(traj_ax_cutted, acc_ax_cutted, delay);

    Prediction_Layer_lambda.Estimate(psi_measure);
    Prediction_Layer_lambda.Update_lambda(lambda, psi_measure, psi, Prediction_Layer_lambda.Estimation);

    // Prediction_Layer_alpha.Estimate(psi_measure);
    // Prediction_Layer_alpha.Update_alpha(alpha,psi_measure,psi,Prediction_Layer_alpha.Estimation);

    // Estimation

    traj_ax_cutted = traj_ax.segment(N_in - N_1, N_1);
    acc_ax_cutted = acc_ax.segment(N_in - N_1, N_1);

    psi_measure = TrajMinJerk.compute_psi(traj_ax_cutted, acc_ax_cutted, delay);
    // Prediction_Layer_alpha.Estimate(psi_measure);
    Prediction_Layer_lambda.Estimate(psi_measure);

    Eigen::VectorXd psi_rls_corr = Prediction_Layer_lambda.Estimation;

    Eigen::VectorXd psi_filtered = psi_rls_corr;
    // Eigen::VectorXd psi_filtered = psi_measure;

    if(Use_Kalman_Filter)
    {
      if(axis == 0)
      {
        F_x.update(psi_rls_corr);
        psi_filtered = F_x.state();
      }
      if(axis == 1)
      {
        F_y.update(psi_rls_corr);
        psi_filtered = F_y.state();
      }
      if(axis == 2)
      {
        F_z.update(psi_rls_corr);
        psi_filtered = F_z.state();
      }
    }

    if(psi_filtered(5) - traj_ax(N_in - 1) > MAX_DIST)
    {

      psi_filtered(7) = 0;
      psi_filtered(6) = 0;
      psi_filtered(5) = traj_ax(N_in - 1) + MAX_DIST;
    }
    if(psi_filtered(5) - traj_ax(N_in - 1) < -MAX_DIST)
    {

      psi_filtered(7) = 0;
      psi_filtered(6) = 0;
      psi_filtered(5) = traj_ax(N_in - 1) - MAX_DIST;
    }

    TrajMinJerk.set_C_from_psi(psi_filtered);
    coeffs_.row(axis) = TrajMinJerk.get_c();
  }
  // std::chrono::duration<double, std::milli> time_span = std::chrono::high_resolution_clock::now() - t_clock;
  // std::cout << "elapsed time " << time_span.count() << std::endl;
}

Eigen::Vector3d Motion_Prediction::get_prediction_Pos_coordinate(double t)
{

  Eigen::Vector3d Output;
  t = std::max(0., std::min(t, t2));
  for(int ax = 0; ax < 3; ax++)
  {
    TrajMinJerk.set_c(coeffs_.row(ax));
    Output[ax] = TrajMinJerk.get_pose(t1 + t);
  }

  // std::cout << "Estimation : " << Output << std::endl;

  return Output;
}

Eigen::Vector3d Motion_Prediction::get_prediction_Vel_coordinate(double t)
{

  Eigen::Vector3d Output;
  t = std::max(0., std::min(t, t2));
  for(int ax = 0; ax < 3; ax++)
  {
    TrajMinJerk.set_c(coeffs_.row(ax));
    Output[ax] = TrajMinJerk.get_vel(t1 + t);
  }

  // std::cout << "Estimation : " << Output << std::endl;

  return Output;
}

Eigen::Vector3d Motion_Prediction::get_prediction_Acc_coordinate(double t)
{
  Eigen::Vector3d Output;
  t = std::max(0., std::min(t, t2));
  for(int ax = 0; ax < 3; ax++)
  {
    TrajMinJerk.set_c(coeffs_.row(ax));
    Output[ax] = TrajMinJerk.get_acc(t1 + t);
  }

  // std::cout << "Estimation : " << Output << std::endl;

  return Output;
}
