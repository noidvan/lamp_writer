#include <igl/get_seconds.h>
#include <igl/opengl/glfw/Viewer.h>

#include <iostream>
#include <vector>

#include "Bone.h"
#include "Skeleton.h"
#include "catmull_rom_interpolation.h"
#include "end_effectors_objective_and_gradient.h"
#include "forward_kinematics.h"
#include "linear_blend_skinning.h"
#include "projected_gradient_descent.h"
#include "read_model_and_rig_from_json.h"
#include "skeleton_visualization_mesh.h"
#include "transformed_tips.h"

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path-to-json-file>" << "\n";
    std::cerr << "Example: " << argv[0] << " ./data/ikea-lamp-drawn-csc317.json"
              << "\n";
    return 1;
  }

  typedef Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
      MapRXd;
  igl::opengl::glfw::Viewer v;
  Eigen::MatrixXd vertices, deformed_vertices, weights;
  Eigen::MatrixXi faces;
  Skeleton skeleton;

  // list of indices into skeleton of bones whose tips are constrained during IK
  Eigen::VectorXi b;
  std::vector<std::vector<std::pair<double, Eigen::Vector3d> > > fk_anim;
  std::vector<std::vector<std::pair<double, Eigen::Vector3d> > >
      fk_anim_positions;
  std::vector<std::pair<double, Eigen::Vector3d> > ik_anim;

  // read mesh, skeleton and weights
  read_model_and_rig_from_json(argv[1], vertices, faces, skeleton, weights,
                               fk_anim, fk_anim_positions, ik_anim, b);

  // If not provided use last bone;
  if (b.size() == 0 && !skeleton.empty()) {
    const int last_bone = static_cast<int>(skeleton.size() - 1);
    b.setConstant(1, 1, last_bone);
  }

  // endpoint positions in a single column
  Eigen::VectorXd xb0 = transformed_tips(skeleton, b);

  // Trail for IK mode visualization
  std::vector<Eigen::Vector3d> trail_points;
  const int max_trail_points = 1000;  // Maximum number of trail points to keep

  // deformed_vertices will track the deforming mesh
  deformed_vertices = vertices;
  const int model_id = 0;
  // skeleton after so dots can be on top
  const int skeleton_id = 1;
  const int trail_id = 2;
  v.append_mesh();
  v.append_mesh();
  v.selected_data_index = 0;
  v.data_list[model_id].set_mesh(deformed_vertices, faces);
  v.data_list[model_id].show_faces = false;
  v.data_list[model_id].set_face_based(true);
  // Color the model based on linear blend skinning weights
  {
    Eigen::MatrixXd color_map =
        (Eigen::MatrixXd(8, 3) << 228, 26, 28, 55, 126, 184, 77, 175, 74, 152,
         78, 163, 255, 127, 0, 255, 255, 51, 166, 86, 40, 247, 129, 191)
            .finished() /
        255.0;
    Eigen::MatrixXd vertex_colors =
        weights *
        color_map
            .replicate(
                (weights.cols() + color_map.rows() - 1) / color_map.rows(), 1)
            .topRows(weights.cols());
    Eigen::MatrixXd face_colors =
        Eigen::MatrixXd::Zero(faces.rows(), vertex_colors.cols());
    for (int i = 0; i < faces.rows(); ++i) {
      for (int j = 0; j < faces.cols(); ++j) {
        face_colors.row(i) += vertex_colors.row(faces(i, j));
      }
    }
    face_colors.array() /= static_cast<double>(faces.cols());
    v.data_list[model_id].set_colors(face_colors);
  }
  // Create a mesh to visualize the skeleton
  Eigen::MatrixXd skeleton_vertices, skeleton_colors;
  Eigen::MatrixXi skeleton_faces;
  const double thickness =
      0.01 *
      (vertices.colwise().maxCoeff() - vertices.colwise().minCoeff()).norm();
  skeleton_visualization_mesh(skeleton, thickness, skeleton_vertices,
                              skeleton_faces, skeleton_colors);
  v.data_list[skeleton_id].set_mesh(skeleton_vertices, skeleton_faces);
  v.data_list[skeleton_id].set_colors(skeleton_colors);
  v.data_list[skeleton_id].set_face_based(true);
  // Initialize trail mesh layer
  v.data_list[trail_id].clear();
  v.data_list[trail_id].line_width = 3.0f;
  v.data_list[trail_id].show_overlay_depth = false;  // Always render on top
  v.core().background_color << 0.05f, 0.1f, 0.2f,
      1.0f;  // Dark navy blue background (complements orange trail)
  v.core().animation_max_fps = 30.;
  v.core().is_animating = true;

  double anim_last_t = igl::get_seconds();
  double anim_t = 0;
  // Update the skeleton mesh and the linear blend skinning model based on
  // current skeleton deformation
  const auto update = [&]() {
    skeleton_visualization_mesh(skeleton, thickness, skeleton_vertices,
                                skeleton_faces, skeleton_colors);
    v.data_list[skeleton_id].set_mesh(skeleton_vertices, skeleton_faces);
    v.data_list[skeleton_id].compute_normals();
    v.data_list[skeleton_id].set_colors(skeleton_colors);
    // Compute transformations of skeleton bones
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> >
        transform;
    forward_kinematics(skeleton, transform);
    // Apply bone transformations to deform shape
    linear_blend_skinning(vertices, skeleton, transform, weights,
                          deformed_vertices);
    v.data_list[model_id].set_vertices(deformed_vertices);
    v.data_list[model_id].compute_normals();
  };

  // Update trail visualization
  const auto update_trail = [&]() {
    if (trail_points.size() < 2) {
      v.data_list[trail_id].clear();
      return;
    }

    // Build line segments from trail points
    int n = static_cast<int>(trail_points.size());
    Eigen::MatrixXd trail_v(n, 3);
    for (int i = 0; i < n; i++) {
      trail_v.row(i) = trail_points[i];
    }

    Eigen::MatrixXi trail_e(n - 1, 2);
    for (int i = 0; i < n - 1; i++) {
      trail_e(i, 0) = i;
      trail_e(i, 1) = i + 1;
    }

    // Orange trail color
    Eigen::RowVector3d orange(1.0, 0.5, 0.0);
    Eigen::MatrixXd trail_c = orange.replicate(n - 1, 1);

    v.data_list[trail_id].clear();
    v.data_list[trail_id].set_edges(trail_v, trail_e, trail_c);
    v.data_list[trail_id].line_width = 3.0f;
  };

  const auto ik = [&]() {
    // If in debug mode use 1 ik iteration per drawn frame, otherwise 100
    const int max_iters =
#if NDEBUG
        200;
#else
        1;
#endif
    // Gather initial angles
    Eigen::VectorXd angle(skeleton.size() * 3);
    for (int si = 0; si < skeleton.size(); si++) {
      angle.block(static_cast<Eigen::Index>(si) * 3, 0, 3, 1) =
          skeleton[si].xzx;
    }
    std::function<double(const Eigen::VectorXd &)> f;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> grad_f;
    std::function<void(Eigen::VectorXd &)> proj_z;
    end_effectors_objective_and_gradient(skeleton, b, xb0, f, grad_f, proj_z);
    // Optimize angles
    projected_gradient_descent(f, grad_f, proj_z, max_iters, angle);
    // Distribute optimized angles
    for (int si = 0; si < skeleton.size(); si++) {
      skeleton[si].xzx =
          angle.block(static_cast<Eigen::Index>(si) * 3, 0, 3, 1);
    }
  };

  v.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool {
    // Check if IK animation data exists
    if (!ik_anim.empty() && v.core().is_animating) {
      const double now = igl::get_seconds();
      anim_t += now - anim_last_t;
      anim_last_t = now;

      // Interpolate IK target position
      Eigen::Vector3d target_pos = catmull_rom_interpolation(ik_anim, anim_t);

      // Update the target for the IK solver
      // Assuming single end effector (lamp head)
      xb0(0) = target_pos(0);
      xb0(1) = target_pos(1);
      xb0(2) = target_pos(2);
    }
    ik();

    // Add current end effector position to trail
    if (v.core().is_animating && b.size() > 0) {
      Eigen::VectorXd xb = transformed_tips(skeleton, b);
      if (xb.size() >= 3) {
        Eigen::Vector3d current_pos(xb(0), xb(1), xb(2));
        trail_points.push_back(current_pos);

        // Limit trail length
        if (trail_points.size() > max_trail_points) {
          trail_points.erase(trail_points.begin());
        }

        // Update trail visualization
        update_trail();
      }
    }
    update();
    return false;
  };

  v.callback_key_pressed = [&](igl::opengl::glfw::Viewer &v, unsigned char key,
                               int /*modifier*/
                               ) -> bool {
    switch (key) {
      default:
        return false;
      case 'R':
      case 'r':
        // Reset animation
        for (auto &bone : skeleton) {
          bone.xzx.setConstant(0);
        }
        xb0 = transformed_tips(skeleton, b);
        anim_last_t = igl::get_seconds();
        anim_t = 0;
        // Clear the trail
        trail_points.clear();
        v.data_list[trail_id].clear();
        update();
        break;
      case ' ':
        v.core().is_animating = !v.core().is_animating;
        if (v.core().is_animating) {
          // Reset clock
          anim_last_t = igl::get_seconds();
        }
        break;
    }
    return true;
  };
  std::cout << R"(
[space]  toggle animation pause/play
R,r      reset animation to beginning
)";
  v.launch();
}
