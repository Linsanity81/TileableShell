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
#include "window.h"
#include "renderer.h"
///////////////////////////////////////////////////////////////////////////////
int main() {
  float width = 1024.0f;
  float height = 768.0f;
  GLWindow w("ShapeOp Demo", width, height);
  GLCHECK //The first error is due to glewInit!
  GLRenderer r(width, height);
  //Ground
  {
    auto g = std::make_shared<Plane>(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
    Eigen::Matrix4f mat = 1000.0 * Eigen::Matrix4f::Identity();
    mat(1, 3) = -0.41;
    mat(3, 3) = 1.0;
    g->setModel(mat);
    r.addMesh(g);
    GLCHECK
  }
  for(int i = 0; i < 10; ++i)
  {
    auto g = std::make_shared<Cube>(Eigen::Vector3f(0.8f, 0.0f, 0.0f));
    Eigen::Matrix4f mat = 0.10*Eigen::Matrix4f::Identity();
    mat(0, 3) = 5.0*std::cos(i);
    mat(1, 3) = -0.32 + 0.02*std::cos(i);
    mat(2, 3) = 5.0*std::sin(i);
    mat(3, 3) = 1.0;
    g->setModel(mat);
    r.addMesh(g);
    GLCHECK
  }
  for(int i = 0; i < 10; ++i)
  {
    auto g = std::make_shared<Cube>(Eigen::Vector3f(0.0f, 0.8f, 0.0f));
    Eigen::Matrix4f mat = 0.10*Eigen::Matrix4f::Identity();
    mat(0, 3) = 5.0*std::cos(i+10);
    mat(1, 3) = -0.32 + 0.02*std::cos(i+10);
    mat(2, 3) = 5.0*std::sin(i+10);
    mat(3, 3) = 1.0;
    g->setModel(mat);
    r.addMesh(g);
    GLCHECK
  }
  for(int i = 0; i < 10; ++i)
  {
    auto g = std::make_shared<Cube>(Eigen::Vector3f(0.0f, 0.0f, 0.8f));
    Eigen::Matrix4f mat = 0.10*Eigen::Matrix4f::Identity();
    mat(0, 3) = 5.0*std::cos(i+20);
    mat(1, 3) = -0.32 + 0.02*std::cos(i+20);
    mat(2, 3) = 5.0*std::sin(i+20);
    mat(3, 3) = 1.0;
    g->setModel(mat);
    r.addMesh(g);
    GLCHECK
  }
  std::vector<std::shared_ptr<ShapeOpSurfaceMesh>> nVec;
  {
    auto n1 = std::make_shared<GLSurfaceMeshObject>("neutral.obj", Eigen::Vector3f(1.0f, 0.5f, 0.5f));
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 3) = -1.4;
    n1->setModel(mat);
    r.addMesh(n1);
    auto s1 = std::make_shared<ShapeOpSurfaceMesh>(n1, 0.01);
    nVec.push_back(s1);
    GLCHECK
  }
  {

    auto n1 = std::make_shared<GLSurfaceMeshObject>("neutral.obj", Eigen::Vector3f(1.0f, 0.8f, 0.5f));
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 3) = -0.7;
    n1->setModel(mat);
    r.addMesh(n1);
    auto s1 = std::make_shared<ShapeOpSurfaceMesh>(n1, 0.1);
    nVec.push_back(s1);
    GLCHECK
  }
  {
    auto n = std::make_shared<GLSurfaceMeshObject>("neutral.obj", Eigen::Vector3f(0.4f, 1.0f, 0.4f));
    r.addMesh(n);
    auto s = std::make_shared<ShapeOpSurfaceMesh>(n, 1.0);
    nVec.push_back(s);
    GLCHECK
  }
  {
    auto n2 = std::make_shared<GLSurfaceMeshObject>("neutral.obj", Eigen::Vector3f(0.5f, 0.8f, 1.0f));
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 3) = 0.7;
    n2->setModel(mat);
    r.addMesh(n2);
    auto s2 = std::make_shared<ShapeOpSurfaceMesh>(n2, 5.0);
    nVec.push_back(s2);
    GLCHECK
  }
  {
    auto n2 = std::make_shared<GLSurfaceMeshObject>("neutral.obj", Eigen::Vector3f(0.5f, 0.5f, 1.0f));
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    mat(0, 3) = 1.4;
    n2->setModel(mat);
    r.addMesh(n2);
    auto s2 = std::make_shared<ShapeOpSurfaceMesh>(n2, 10.0);
    nVec.push_back(s2);
    GLCHECK
  }

  ////
  auto c = std::make_shared<Camera>(Eigen::Vector3f(2.2f, 1.8f, 2.8f), Eigen::Vector3f::Zero(), width / height);
  r.setCamera(c);
  GLCHECK
  auto l = std::make_shared<Camera>(Eigen::Vector3f(2.0f, 4.0f, 2.0f), Eigen::Vector3f::Zero(), width / height);
  r.setLight(l);

  w.loop(std::bind(&GLRenderer::display, &r), nVec, c);
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
