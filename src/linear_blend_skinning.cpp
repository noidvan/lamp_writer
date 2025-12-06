#include "linear_blend_skinning.h"

void linear_blend_skinning(
    const Eigen::MatrixXd &V, const Skeleton &skeleton,
    const std::vector<Eigen::Affine3d,
                      Eigen::aligned_allocator<Eigen::Affine3d> > &T,
    const Eigen::MatrixXd &W, Eigen::MatrixXd &U) {
  U.resize(V.rows(), 3);
  U.setZero();

  // For each vertex
  for (int v = 0; v < V.rows(); v++) {
    Eigen::Vector4d rest_pos(V(v, 0), V(v, 1), V(v, 2), 1.0);
    Eigen::Vector3d pose_pos(0, 0, 0);

    // Sum weighted transformations from all bones
    for (int i = 0; i < skeleton.size(); i++) {
      int weight_idx = skeleton[i].weight_index;
      if (weight_idx >= 0) {
        double weight = W(v, weight_idx);
        Eigen::Vector4d transformed = T[i].matrix() * rest_pos;
        pose_pos += weight * transformed.head<3>();
      }
    }

    U.row(v) = pose_pos;
  }
}
