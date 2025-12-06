#include "projected_gradient_descent.h"

#include "line_search.h"

void projected_gradient_descent(
    const std::function<double(const Eigen::VectorXd &)> &f,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &grad_f,
    const std::function<void(Eigen::VectorXd &)> &proj_z, const int max_iters,
    Eigen::VectorXd &z) {
  const double tolerance = 1e-10;
  const double max_step = 100.0;

  for (int iter = 0; iter < max_iters; iter++) {
    // Compute gradient at current position
    Eigen::VectorXd grad = grad_f(z);

    // Descent direction
    Eigen::VectorXd dz = -grad;

    // Check for convergence
    if (dz.norm() < tolerance) {
      break;
    }

    // Find step size
    double sigma = line_search(f, proj_z, z, dz, max_step);

    // Update position
    z = z + sigma * dz;

    // Project onto constraints
    proj_z(z);
  }
}
