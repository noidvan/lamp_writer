#include "transformed_tips.h"

#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(const Skeleton &skeleton,
                                 const Eigen::VectorXi &b) {
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> t;
  forward_kinematics(skeleton, t);

  Eigen::VectorXd tip_positions(3 * b.size());

  for (int i = 0; i < b.size(); i++) {
    int bone_idx = b(i);
    // Canonical tip at (length, 0, 0) in homogeneous coordinates
    Eigen::Vector4d canonical_tip(skeleton[bone_idx].length, 0, 0, 1);
    // Transform canonical -> rest -> pose
    Eigen::Vector4d rest_tip =
        skeleton[bone_idx].rest_T.matrix() * canonical_tip;
    Eigen::Vector4d pose_tip = t[bone_idx].matrix() * rest_tip;
    // Extract x, y, z coordinates
    tip_positions.segment<3>(static_cast<Eigen::Index>(3) * i) =
        pose_tip.head<3>();
  }

  return tip_positions;
}
