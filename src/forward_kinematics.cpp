#include "forward_kinematics.h"

#include "euler_angles_to_transform.h"

void forward_kinematics(
    const Skeleton &skeleton,
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >
        &T) {
  // T[i] = T[parent] * rest_T[i] * R(xzx[i]) * rest_T[i]^(-1)
  T.resize(skeleton.size(), Eigen::Affine3d::Identity());

  // Process bones in order
  for (int i = 0; i < skeleton.size(); i++) {
    const Bone &bone = skeleton[i];

    Eigen::Affine3d r = euler_angles_to_transform(bone.xzx);

    // Get parent transformation
    Eigen::Affine3d t_parent = Eigen::Affine3d::Identity();
    if (bone.parent_index >= 0) {
      t_parent = T[bone.parent_index];
    }

    // Compute pose transformation
    T[i] = t_parent * bone.rest_T * r * bone.rest_T.inverse();
  }
}
