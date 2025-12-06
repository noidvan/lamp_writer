#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(const Eigen::Vector3d &xzx) {
  const double deg_to_rad = M_PI / 180.0;
  Eigen::Affine3d a = Eigen::Affine3d::Identity();
  a.linear() =
      (Eigen::AngleAxisd(xzx(2) * deg_to_rad, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(xzx(1) * deg_to_rad, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(xzx(0) * deg_to_rad, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return a;
}
