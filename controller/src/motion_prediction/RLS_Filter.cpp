#include "RLS_Filter.h"

void RLS_Filter::Estimate(const Eigen::VectorXd & Input)
{

  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(output_shape, input_shape * output_shape);
  for(int i = 0; i < output_shape; i++) { Phi.block(i, i * input_shape, 1, input_shape) = Input.transpose(); }

  Estimation = Phi * Theta;
  Estimation_Error = Phi * P * Phi.transpose();
}

void RLS_Filter::Update_lambda(const double lambda,
                               const Eigen::VectorXd & Input,
                               const Eigen::VectorXd & Real,
                               const Eigen::VectorXd & Pred)
{

  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(output_shape, input_shape * output_shape);
  for(int i = 0; i < output_shape; i++) { Phi.block(i, i * input_shape, 1, input_shape) = Input.transpose(); }

  P *= lambda;

  Eigen::MatrixXd D =
      ((Phi * P * Phi.transpose() + 1e-12 * Eigen::MatrixXd::Identity(output_shape, output_shape)).inverse());

  K = P * Phi.transpose() * D;

  P = ((Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * Phi) * P);

  Theta += K * (Real - Pred);
}

void RLS_Filter::Update_alpha(const double alpha,
                              const Eigen::VectorXd & Input,
                              const Eigen::VectorXd & Real,
                              const Eigen::VectorXd & Pred)
{

  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(output_shape, input_shape * output_shape);
  for(int i = 0; i < output_shape; i++) { Phi.block(i, i * input_shape, 1, input_shape) = Input.transpose(); }
  P += Rr;

  Eigen::MatrixXd D =
      (Phi * P * Phi.transpose() + 1e-12 * Eigen::MatrixXd::Identity(output_shape, output_shape)).inverse();

  K = P * Phi.transpose() * D;

  P = ((Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * Phi) * P);
  Rr = (1 - alpha) * Rr + alpha * K * (Real - Pred) * (Real - Pred).transpose() * K.transpose();
  Theta += K * (Real - Pred);
}
