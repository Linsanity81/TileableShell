///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014 Sofien Bouaziz <sofien.bouaziz@gmail.com>
// Copyright (C) 2014 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#ifndef HELPER_H
#define HELPER_H
///////////////////////////////////////////////////////////////////////////////
#include <Types.h>
#include <OpenGP/Surface_mesh.h>
///////////////////////////////////////////////////////////////////////////////
void computeMassMatrix(opengp::Surface_mesh &mesh, ShapeOp::SparseMatrix &m) {
  std::vector<ShapeOp::Triplet> triplets;
  auto vertices = mesh.vertex_property<opengp::Vec3>("v:point");
  m = ShapeOp::SparseMatrix(mesh.n_vertices(), mesh.n_vertices());
  Eigen::Map<Eigen::Matrix3Xf> positions = (Eigen::Map<Eigen::Matrix3Xf>((float *)(vertices.data()), 3, mesh.n_vertices()));
  for (auto fit = mesh.faces_begin(); fit != mesh.faces_end(); ++fit) {
    assert(mesh.valence(*fit) == 3);
    std::vector<int> id(3);
    ShapeOp::Matrix33 p;
    auto vit = mesh.vertices(*fit);
    for (unsigned int i = 0; i < 3; ++i) {
      id[i] = (*vit).idx();
      ++vit;
      p.col(i) = positions.col(id[i]).cast<ShapeOp::Scalar>();
    }
    ShapeOp::Scalar a = (p.col(0) - p.col(1)).norm();
    ShapeOp::Scalar b = (p.col(1) - p.col(2)).norm();
    ShapeOp::Scalar c = (p.col(2) - p.col(0)).norm();
    ShapeOp::Scalar s = (a + b + c) / ShapeOp::Scalar(2.0);
    ShapeOp::Scalar A = std::sqrt(s * (s - a) * (s - b) * (s - c));
    for (int d0 = 0; d0 < 3; d0++) {
      triplets.push_back(ShapeOp::Triplet(id[d0], id[d0], A / 3.0));
    }
//        for (int d0=0; d0<3; d0++) {
//            for (int d1=0; d1<3; d1++) {
//                if (d0==d1)
//                    triplets.push_back(ShapeOp::Triplet(id[d0],id[d1], A/10.0));
//                else
//                    triplets.push_back(ShapeOp::Triplet(id[d0],id[d1], A/20.0));
//            }
//        }
  }
  m.setFromTriplets(triplets.begin(), triplets.end());
}
///////////////////////////////////////////////////////////////////////////////
void edgeFuntional(opengp::Surface_mesh &mesh, const std::function<void(const std::vector<int>&)> &f) {
  for (auto eit = mesh.edges_begin(); eit != mesh.edges_end(); ++eit) {
    auto he = mesh.halfedge(*eit, 0);
    std::vector<int> id(2);
    id[0] = mesh.to_vertex(he).idx();
    id[1] = mesh.from_vertex(he).idx();
    f(id);
  }
}
///////////////////////////////////////////////////////////////////////////////
void polygonFuntional(opengp::Surface_mesh &mesh, const std::function<void(const std::vector<int>&)> &f) {
  for (auto fit = mesh.faces_begin(); fit != mesh.faces_end(); ++fit) {
    unsigned int n = mesh.valence(*fit);
    std::vector<int> id(n);
    auto vit = mesh.vertices(*fit);
    for (unsigned int i = 0; i < n; ++i) {
      id[i] = (*vit).idx();
      ++vit;
    }
    f(id);
  }
}
///////////////////////////////////////////////////////////////////////////////
void ringFuntional(opengp::Surface_mesh &mesh, const std::function<void(const std::vector<int>&)> &f) {
  for (auto vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit) {
    unsigned int n = mesh.valence(*vit);
    std::vector<int> id(n + 1);
    id[0] = (*vit).idx();
    auto vvit = mesh.vertices(*vit);
    for (unsigned int i = 0; i < n; ++i) {
      id[i + 1] = (*vvit).idx();
      ++vvit;
    }
    f(id);
  }
}
///////////////////////////////////////////////////////////////////////////////
void trianglePairFuntional(opengp::Surface_mesh &mesh, const std::function<void(const std::vector<int>&)> &f) {
  for (auto eit = mesh.edges_begin(); eit != mesh.edges_end(); ++eit) {
    if (!mesh.is_boundary(*eit)) {
      std::vector<int> id(4);
      auto he = mesh.halfedge(*eit, 0);
      id[0] = mesh.from_vertex(he).idx();
      id[1] = mesh.to_vertex(he).idx();
      auto he1 = mesh.next_halfedge(he);
      id[2] = mesh.to_vertex(he1).idx();
      auto he2 = mesh.cw_rotated_halfedge(he);
      id[3] = mesh.to_vertex(he2).idx();
      f(id);
    }
  }
}
///////////////////////////////////////////////////////////////////////////////
void edgePairFuntional(opengp::Surface_mesh &mesh, const std::function<void(const std::vector<int>&)> &f) {
  for (auto vit = mesh.vertices_begin(); vit != mesh.vertices_end(); ++vit) {
    unsigned int n = mesh.valence(*vit);
    auto r = (*vit).idx();
    auto vvit = mesh.vertices(*vit);
    std::vector<int> v(n);
    for (unsigned int i = 0; i < n; ++i) {
      v[i] = (*vvit).idx();
      ++vvit;
    }
    for (unsigned int i = 0; i < (n - 1); ++i) {
      std::vector<int> id(3);
      id[0] = r;
      id[1] = v[i];
      for (unsigned int j = (i + 1); j < n; ++j) {
        id[2] = v[j];
        f(id);
      }
    }
  }
}
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////

