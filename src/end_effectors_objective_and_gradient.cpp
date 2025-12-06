#include "end_effectors_objective_and_gradient.h"

#include "copy_skeleton_at.h"
#include "kinematics_jacobian.h"
#include "transformed_tips.h"

void end_effectors_objective_and_gradient(
    const Skeleton &skeleton, const Eigen::VectorXi &b,
    const Eigen::VectorXd &xb0,
    std::function<double(const Eigen::VectorXd &)> &f,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &grad_f,
    std::function<void(Eigen::VectorXd &)> &proj_z) {
  // Objective function
  f = [&](const Eigen::VectorXd &A) -> double {
    Skeleton skel = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd x = transformed_tips(skel, b);
    Eigen::VectorXd diff = x - xb0;
    return diff.squaredNorm();
  };

  // Gradient function
  grad_f = [&](const Eigen::VectorXd &A) -> Eigen::VectorXd {
    Skeleton skel = copy_skeleton_at(skeleton, A);
    Eigen::VectorXd x = transformed_tips(skel, b);
    Eigen::VectorXd diff = x - xb0;

    Eigen::MatrixXd j;
    kinematics_jacobian(skel, b, j);

    return 2.0 * j.transpose() * diff;
  };

  // Projection function
  proj_z = [&](Eigen::VectorXd &A) {
    assert(skeleton.size() * 3 == A.size());
    for (int i = 0; i < skeleton.size(); i++) {
      for (int j = 0; j < 3; j++) {
        int idx = 3 * i + j;
        A(idx) = std::max(skeleton[i].xzx_min(j),
                          std::min(skeleton[i].xzx_max(j), A(idx)));
      }
    }
  };
}
