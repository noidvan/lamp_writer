#include "copy_skeleton_at.h"

Skeleton copy_skeleton_at(const Skeleton &skeleton, const Eigen::VectorXd &A) {
  Skeleton copy = skeleton;

  for (int i = 0; i < skeleton.size(); i++) {
    copy[i].xzx = A.segment<3>(static_cast<Eigen::Index>(3) * i);
  }

  return copy;
}
