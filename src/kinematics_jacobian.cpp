#include "kinematics_jacobian.h"

#include "copy_skeleton_at.h"
#include "transformed_tips.h"

void kinematics_jacobian(const Skeleton &skeleton, const Eigen::VectorXi &b,
                         Eigen::MatrixXd &J) {
  const double h = 1e-7;
  J.resize(static_cast<Eigen::Index>(b.size() * 3),
           static_cast<Eigen::Index>(skeleton.size() * 3));
  J.setZero();

  // Get current tip positions
  Eigen::VectorXd x0 = transformed_tips(skeleton, b);

  // Create angle vector from current skeleton
  Eigen::VectorXd angles(static_cast<Eigen::Index>(skeleton.size() * 3));
  for (int i = 0; i < static_cast<int>(skeleton.size()); i++) {
    angles.segment<3>(static_cast<Eigen::Index>(3) * i) = skeleton[i].xzx;
  }

  // For each angle parameter
  for (int j = 0; j < static_cast<int>(skeleton.size() * 3); j++) {
    Eigen::VectorXd angles_perturbed = angles;
    angles_perturbed(j) += h;

    // Create skeleton with perturbed angles
    Skeleton skeleton_perturbed = copy_skeleton_at(skeleton, angles_perturbed);

    // Compute tip positions with perturbed angle
    Eigen::VectorXd x_perturbed = transformed_tips(skeleton_perturbed, b);

    // Compute partial derivatives
    J.col(j) = (x_perturbed - x0) / h;
  }
}
