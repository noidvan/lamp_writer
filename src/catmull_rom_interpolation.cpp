#include "catmull_rom_interpolation.h"

#include <Eigen/Dense>
#include <algorithm>

Eigen::Vector3d catmull_rom_interpolation(
    const std::vector<std::pair<double, Eigen::Vector3d> > &keyframes,
    double t) {
  if (keyframes.empty()) {
    return Eigen::Vector3d(0, 0, 0);
  }

  // Before first or after last keyframe
  if (t <= keyframes.front().first) {
    return keyframes.front().second;
  }
  if (t >= keyframes.back().first) {
    return keyframes.back().second;
  }

  // Find the segment containing t (between P1 and P2)
  int i1 = 0;
  for (int i = 0; i < keyframes.size() - 1; i++) {
    if (t >= keyframes[i].first && t < keyframes[i + 1].first) {
      i1 = i;
      break;
    }
  }

  int i0 = std::max(0, i1 - 1);
  int i2 = std::min((int)keyframes.size() - 1, i1 + 1);
  int i3 = std::min((int)keyframes.size() - 1, i1 + 2);

  // Get the four control points
  Eigen::Vector3d p0 = keyframes[i0].second;
  Eigen::Vector3d p1 = keyframes[i1].second;
  Eigen::Vector3d p2 = keyframes[i2].second;
  Eigen::Vector3d p3 = keyframes[i3].second;

  // Normalize t to [0, 1] between P1 and P2
  double t1 = keyframes[i1].first;
  double t2 = keyframes[i2].first;
  double u = (t - t1) / (t2 - t1);

  // Catmull-Rom
  double u2 = u * u;
  double u3 = u2 * u;

  Eigen::Vector3d result = 0.5 * (2.0 * p1 + (-p0 + p2) * u +
                                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2 +
                                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3);

  return result;
}
