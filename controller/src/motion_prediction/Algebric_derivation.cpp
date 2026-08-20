#include "Algebric_derivation.h"
#include <Eigen/Dense>

double algebric_derivator(const Eigen::VectorXd & x, const double freq, const int deriv)
{

  int n = x.size();
  Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(n, 0, ((double)n) / freq);
  double T = t(n - 1);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
  double f_sum = 0;
  if(deriv == 1)
  {

    for(int i = 0; i < n; i++)
    {

      f(i) = 6 * ((2 * t(i) - T) * x(i)) / (std::pow(T, 3));
      f_sum += f(i);
    }
  }

  if(deriv == 2)
  {

    for(int i = 0; i < n; i++)
    {

      f(i) = (tgamma(6) / (2 * std::pow(T, 5))) * (std::pow(T - t(i), 2) - t(i) * (4 * (T - t(i))) + std::pow(t(i), 2))
             * x(i); // tgamma(i+1) = i!
      f_sum += f(i);
    }
  }

  return (f_sum / freq) - (0.5 / freq) * (f(0) + f(n - 1));
}

Eigen::VectorXd algebric_derivate(const Eigen::VectorXd & X, const int n_window, const double freq, const int deriv)
{
  int n_X = X.size();
  int n_out = n_X - n_window;

  Eigen::VectorXd Out = Eigen::VectorXd::Zero(n_out);

  for(int i = 0; i < n_out; i++)
  {

    Eigen::VectorXd x = X.segment(i, n_window);
    Out(i) = algebric_derivator(x, freq, deriv);
  }

  return Out;
}
