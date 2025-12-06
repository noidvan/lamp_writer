#ifndef READ_MODEL_AND_RIG_FROM_JSON_H
#define READ_MODEL_AND_RIG_FROM_JSON_H

#include <Eigen/Core>
#include <utility>  // std::pair
#include <vector>

#include "Skeleton.h"
// Read a model mesh, a skeleton + rest positions/orientations + joint limits
// and linear blend skinning weights.
//
// Inputs:
//   json_file  path to .json file
// Outputs:
//   V  #V by 3 list of vertex positions
//   F  #F by 3 list of triangle indices into V
//   skeleton  #bones list of bones
//   W  #V by #W list of skinning weights
//   fk_anim  #bones list of list of forward kinematics keyframes
//     (times+relative bone rotations)
//   fk_anim_positions  #bones list of list of position keyframes
//     (times+absolute bone positions)
//   ik_anim  list of IK target positions over time (times+3D positions)
//   b  #b list of inverse kinematices constraint indices into list of bones
// Returns true iff readable file exists
bool read_model_and_rig_from_json(
    const std::string &filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F,
    Skeleton &skeleton, Eigen::MatrixXd &W,
    std::vector<std::vector<std::pair<double, Eigen::Vector3d> > > &fk_anim,
    std::vector<std::vector<std::pair<double, Eigen::Vector3d> > >
        &fk_anim_positions,
    std::vector<std::pair<double, Eigen::Vector3d> > &ik_anim,
    Eigen::VectorXi &b);

// Implementation

#include <igl/dirname.h>
#include <igl/readDMAT.h>
#include <igl/read_triangle_mesh.h>

#include <Eigen/Geometry>
#include <cassert>
#include <fstream>
#include <iostream>
#include <json.hpp>

bool read_model_and_rig_from_json(
    const std::string &filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F,
    Skeleton &skeleton, Eigen::MatrixXd &W,
    std::vector<std::vector<std::pair<double, Eigen::Vector3d> > > &fk_anim,
    std::vector<std::vector<std::pair<double, Eigen::Vector3d> > >
        &fk_anim_positions,
    std::vector<std::pair<double, Eigen::Vector3d> > &ik_anim,
    Eigen::VectorXi &b) {
  // Heavily borrowing from
  // https://github.com/yig/graphics101-raycasting/blob/master/parser.cpp
  using json = nlohmann::json;

  std::ifstream infile(filename);
  if (!infile) {
    return false;
  }
  json j;
  infile >> j;

  auto parse_affine = [](const json &j) -> Eigen::Affine3d {
    return Eigen::Affine3d((Eigen::Matrix4d(4, 4) << j[0][0], j[0][1], j[0][2],
                            j[0][3], j[1][0], j[1][1], j[1][2], j[1][3],
                            j[2][0], j[2][1], j[2][2], j[2][3], 0, 0, 0, 1)
                               .finished());
  };
#if defined(WIN32) || defined(_WIN32)
#define PATH_SEPARATOR std::string("\\")
#else
#define PATH_SEPARATOR std::string("/")
#endif
  const std::string dir = igl::dirname(filename) + PATH_SEPARATOR;
  // Parse mesh
  std::string mesh_filename = j["model"]["file"];
  igl::read_triangle_mesh(dir + mesh_filename, V, F);
  // Parse weights
  std::string weights_filename = j["weights"]["file"];
  igl::readDMAT(dir + weights_filename, W);
  std::cout << dir + weights_filename << '\n';
  // parse bones
  skeleton.clear();
  const auto num_bones = static_cast<size_t>(j["bones"].size());
  skeleton.reserve(num_bones);
  for (const json &jbone : j["bones"]) {
    skeleton.emplace_back(jbone["parent_id"], jbone["weight_id"],
                          parse_affine(jbone["rest_T"]), jbone["length"]);
    if (jbone.count("xzx_min")) {
      skeleton.back().xzx_min = Eigen::Vector3d(
          jbone["xzx_min"][0], jbone["xzx_min"][1], jbone["xzx_min"][2]);
    }
    if (jbone.count("xzx_max")) {
      const json &jmax = jbone["xzx_max"];
      skeleton.back().xzx_max = Eigen::Vector3d(jmax[0], jmax[1], jmax[2]);
    }
  }
  if (j.count("fk_animation")) {
    fk_anim.resize(num_bones);
    fk_anim_positions.resize(num_bones);
    for (const json &jfk_bone : j["fk_animation"]) {
      const int bi = jfk_bone["bone_id"];
      for (const json &jkey : jfk_bone["keys"]) {
        fk_anim[bi].emplace_back(
            jkey["t"],
            Eigen::Vector3d(jkey["xzx"][0], jkey["xzx"][1], jkey["xzx"][2]));
        // Parse position keyframes if present
        if (jkey.count("position")) {
          const json &jpos = jkey["position"];
          fk_anim_positions[bi].emplace_back(
              jkey["t"], Eigen::Vector3d(jpos[0], jpos[1], jpos[2]));
        }
      }
      // assuming sorted by t, otherwise should do a sort here.
    }
  }
  if (j.count("ik_animation")) {
    for (const json &jkey : j["ik_animation"]) {
      const json &jtarget = jkey["target"];
      ik_anim.emplace_back(jkey["t"],
                           Eigen::Vector3d(jtarget[0], jtarget[1], jtarget[2]));
    }
  }
  if (j.count("ik_constraints")) {
    b.resize(static_cast<Eigen::Index>(j["ik_constraints"].size()));
    if (j["ik_constraints"].size() == 1 && !j["ik_constraints"].is_array()) {
      b(0) = j["ik_constraints"];
    } else {
      for (Eigen::Index i = 0; i < b.size(); i++) {
        b(i) = j["ik_constraints"][i];
      }
    }
  }
  return true;
}

#endif
