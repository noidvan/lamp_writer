#ifndef SKELETON_VISUALIZATION_MESH_H
#define SKELETON_VISUALIZATION_MESH_H

#include <Eigen/Core>

#include "Skeleton.h"

// Visualize a skeleton as a 3D mesh using long oriented pyramids to represent
// the bones.
//
// Inputs:
//   skeleton  #bones list of bone objects
//   thickness  amount to scale in non-axial directions
// Outputs:
//   SV  #SV by 3 list of mesh vertex positions
//   SF  #SF by 3 list of triangle indices into SV
//   SV  #SF by 3 list of face colors
void skeleton_visualization_mesh(const Skeleton &skeleton, double thickness,
                                 Eigen::MatrixXd &SV, Eigen::MatrixXi &SF,
                                 Eigen::MatrixXd &SC);

// Implementation
#include "forward_kinematics.h"

void skeleton_visualization_mesh(const Skeleton &skeleton,
                                 const double thickness, Eigen::MatrixXd &SV,
                                 Eigen::MatrixXi &SF, Eigen::MatrixXd &SC) {
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > t;
  forward_kinematics(skeleton, t);
  Eigen::MatrixXd bv(5, 3);
  bv << 0, -1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 1, 0, 0;
  bv.rightCols(2) *= thickness;
  Eigen::MatrixXi bf(6, 3);
  bf << 0, 2, 1, 0, 3, 2, 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4;
  Eigen::MatrixXd bc(6, 3);
  Eigen::RowVector3d red(1, 0, 0), green(0, 1, 0), blue(0, 0, 1);
  bc << 1 - red.array(), 1 - red.array(), 1 - blue.array(), green, blue,
      1 - green.array();
  int num_bones = 0;
  for (int b = 0; b < skeleton.size(); b++) {
    if (skeleton[b].parent_index >= 0) {
      num_bones++;
    }
  }
  SV.resize(bv.rows() * num_bones, 3);
  SF.resize(bf.rows() * num_bones, 3);
  SC.resize(bf.rows() * num_bones, 3);
  {
    int k = 0;
    for (int b = 0; b < skeleton.size(); b++) {
      if (skeleton[b].parent_index < 0) {
        continue;
      }
      const Eigen::Affine3d &r_tp = skeleton[skeleton[b].parent_index].rest_T;
      const Eigen::Affine3d &r_tb = skeleton[b].rest_T;
      const double len = skeleton[b].length;
      Eigen::MatrixXd bvk(bv.rows(), 3);
      for (int v = 0; v < bv.rows(); v++) {
        const Eigen::Vector3d p =
            t[b] * (r_tb * Eigen::Vector3d(len * bv(v, 0), bv(v, 1), bv(v, 2)));
        bvk.row(v) = p.transpose();
      }
      SV.block(k * bv.rows(), 0, bv.rows(), 3) = bvk;
      SF.block(k * bf.rows(), 0, bf.rows(), 3) =
          (bf.array() + k * bv.rows()).matrix();
      SC.block(k * bc.rows(), 0, bc.rows(), 3) = bc;
      k++;
    }
  }
}

#endif
