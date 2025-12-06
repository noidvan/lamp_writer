#include "line_search.h"

double line_search(const std::function<double(const Eigen::VectorXd &)> &f,
                   const std::function<void(Eigen::VectorXd &)> &proj_z,
                   const Eigen::VectorXd &z, const Eigen::VectorXd &dz,
                   const double max_step) {
  double sigma = max_step;
  const double min_step = 1e-8;
  double f_current = f(z);

  while (sigma > min_step) {
    // Compute proposed new position
    Eigen::VectorXd z_new = z + sigma * dz;
    // Project onto constraints
    proj_z(z_new);

    // Check if objective decreases
    double f_new = f(z_new);
    if (f_new < f_current) {
      return sigma;
    }

    // Reduce step size
    sigma *= 0.5;
  }

  return sigma;
}
