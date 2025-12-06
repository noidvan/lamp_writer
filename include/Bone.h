#ifndef BONE_H
#define BONE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>

class Bone {
  const double inf_ = std::numeric_limits<double>::infinity();

 public:
  // Index into skeleton bone-list  of parent (-1 indicates root)
  int parent_index;
  // Index into columns of weights matrix for corresponding linear blend
  // skinning weights (-1 indicates no associated weights)
  int weight_index;
  // Transformation mapping from "canonical bone" to "rest bone"
  Eigen::Affine3d rest_T;
  // Length of bone.
  double length;
  // Euler Angles of current pose
  Eigen::Vector3d xzx;
  // max, min Euler Angles: joint limits
  Eigen::Vector3d xzx_max, xzx_min;
  Bone(int _parent_index, int _weight_index, Eigen::Affine3d _rest_T,
       double _length)
      : rest_T(std::move(_rest_T)),
        length(_length),
        parent_index(_parent_index),
        weight_index(_weight_index),
        xzx(0, 0, 0),
        xzx_max(inf_, inf_, inf_),
        xzx_min(-inf_, -inf_, -inf_) {};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
