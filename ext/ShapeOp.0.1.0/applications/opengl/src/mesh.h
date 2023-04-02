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
#ifndef MESH_H
#define MESH_H
///////////////////////////////////////////////////////////////////////////////
#include <Types.h>
#include <GL/glew.h>
#include <GL/glfw.h>
#include <OpenGP/Surface_mesh.h>
#include <OpenGP/surface_mesh/IO.h>
#include "helper.h"
#include "Solver.h"
#include "Constraint.h"
#include "Force.h"
///////////////////////////////////////////////////////////////////////////////
/** \brief An abstract base class for renderable objects.*/
class Object {
 public:
  Object(const Eigen::Matrix4f &model,
         const Eigen::Vector3f &color) : model_(model), color_(color) {}
  virtual ~Object() {}
  virtual void display() = 0;
  void setModel(const Eigen::Matrix4f &model) { model_ = model; }
  const Eigen::Matrix4f &getModel() const { return model_; }
  const Eigen::Vector3f &getColor() const { return color_; }
 protected:
  Eigen::Matrix4f model_;
  Eigen::Vector3f color_;
  GLuint vertexArrayID_;
  GLuint vertexBuffer_;
  GLuint normalBuffer_;
  int numIndices_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A renderable Plane.*/
class Plane : public Object {
 public:
  Plane(const Eigen::Vector3f &color = Eigen::Vector3f(1.0f, 0.0f, 0.0f)) : Object(Eigen::Matrix4f::Identity(), color) {
    init();
  }
  void display() {
    glBindVertexArray(vertexArrayID_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
  }
 private:
  void init() {
    /// Vertex Array
    glGenVertexArrays(1, &vertexArrayID_);
    glBindVertexArray(vertexArrayID_);
    /// Vertex Buffer
    GLfloat position[] = {
      -1.0f, 0.0f, -1.0f,
      1.0f, 0.0f, -1.0f,
      -1.0f,  0.0f, 1.0f,
      -1.0f,  0.0f, 1.0f,
      1.0f, 0.0f, -1.0f,
      1.0f,  0.0f, 1.0f,
    };
    GLfloat normal[] = {
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
    };
    glGenBuffers(1, &vertexBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(position) , position, GL_STATIC_DRAW);
    /// Vertex Attribute ID
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &normalBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(normal) , normal, GL_STATIC_DRAW);
    /// Vertex Attribute ID
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
  }
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A renderable Cube.*/
class Cube : public Object {
 public:
  Cube(const Eigen::Vector3f &color = Eigen::Vector3f(1.0f, 0.0f, 0.0f)) : Object(Eigen::Matrix4f::Identity(), color) {
    init();
  }
  void display() {
    glBindVertexArray(vertexArrayID_);
    glDrawArrays(GL_TRIANGLES, 0, 36);
  }
 private:
  void init() {
    /// Vertex Array
    glGenVertexArrays(1, &vertexArrayID_);
    glBindVertexArray(vertexArrayID_);
    /// Vertex Buffer
    GLfloat position[] = {
      -1.0f, 1.0f, -1.0f,
      1.0f, 1.0f, -1.0f,
      -1.0f,  1.0f, 1.0f,
      -1.0f,  1.0f, 1.0f,
      1.0f, 1.0f, -1.0f,
      1.0f,  1.0f, 1.0f,

      -1.0f, -1.0f, -1.0f,
      1.0f, -1.0f, -1.0f,
      -1.0f,  -1.0f, 1.0f,
      -1.0f,  -1.0f, 1.0f,
      1.0f, -1.0f, -1.0f,
      1.0f,  -1.0f, 1.0f,

      1.0f, -1.0f, -1.0f,
      1.0f, 1.0f, -1.0f,
      1.0f, -1.0f, 1.0f,
      1.0f, -1.0f, 1.0f,
      1.0f, 1.0f, -1.0f,
      1.0f,  1.0f, 1.0f,

      -1.0f, -1.0f, -1.0f,
      -1.0f, 1.0f,  -1.0f,
      -1.0f, -1.0f, 1.0f,
      -1.0f, -1.0f, 1.0f,
      -1.0f, 1.0f,  -1.0f,
      -1.0f, 1.0f,   1.0f,

      -1.0f, -1.0f, 1.0f,
      1.0f, -1.0f, 1.0f,
      -1.0f,  1.0f, 1.0f,
      -1.0f,  1.0f, 1.0f,
      1.0f, -1.0f, 1.0f,
      1.0f,  1.0f, 1.0f,

      -1.0f, -1.0f, -1.0f,
      1.0f, -1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
      1.0f, -1.0f, -1.0f,
      1.0f,  1.0f, -1.0f,
    };
    GLfloat normal[] = {
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f,

      0.0f, -1.0f, 0.0f,
      0.0f, -1.0f, 0.0f,
      0.0f, -1.0f, 0.0f,
      0.0f, -1.0f, 0.0f,
      0.0f, -1.0f, 0.0f,
      0.0f, -1.0f, 0.0f,

      1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,

      -1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f,
      -1.0f, 0.0f, 0.0f,

      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f,
      0.0f, 0.0f, 1.0f,

      0.0f, 0.0f, -1.0f,
      0.0f, 0.0f, -1.0f,
      0.0f, 0.0f, -1.0f,
      0.0f, 0.0f, -1.0f,
      0.0f, 0.0f, -1.0f,
      0.0f, 0.0f, -1.0f,
    };
    glGenBuffers(1, &vertexBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(position) , position, GL_STATIC_DRAW);
    /// Vertex Attribute ID
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &normalBuffer_);
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(normal) , normal, GL_STATIC_DRAW);
    /// Vertex Attribute ID
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
  }
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A renderable SurfaceMesh.*/
class GLSurfaceMeshObject : public Object {
 public:
  GLSurfaceMeshObject(const std::string &file,
                      const Eigen::Vector3f &color = Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                      bool update = true) : Object(Eigen::Matrix4f::Identity(), color) {
    opengp::read_mesh(mesh_, file);
    init(update);
  }
  virtual void display() override final {
    glBindVertexArray(vertexArrayID_);
    glDrawElements(GL_TRIANGLES, numIndices_, GL_UNSIGNED_INT, 0);
  }
  void update() {
    auto vnormal = mesh_.get_vertex_property<opengp::Vec3>("v:normal");
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(mesh_.n_vertices()); ++i)
      vnormal[opengp::Surface_mesh::Vertex(i)] = mesh_.compute_vertex_normal(opengp::Surface_mesh::Vertex(i));

    glBindVertexArray(vertexArrayID_);
    updateVertexBuffer(false);
    updateNormalBuffer(false);
  }
  opengp::Surface_mesh &getMesh() {
    return mesh_;
  }
 private:
  void updateVertexBuffer(bool malloc = true) {
    auto vpoints = mesh_.get_vertex_property<opengp::Vec3>("v:point");
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    if (malloc) {
      glBufferData(GL_ARRAY_BUFFER, mesh_.n_vertices() * sizeof(opengp::Vec3), vpoints.data(), GL_STATIC_DRAW);
    } else {
      GLfloat *data = (GLfloat *) glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
      float *d  = (float *) vpoints.data();
      #pragma omp parallel for
      for (int i = 0; i < static_cast<int>(mesh_.n_vertices() * 3); ++i) {
        data[i] = d[i];
      }
      glUnmapBuffer(GL_ARRAY_BUFFER);
    }
  }
  void updateNormalBuffer(bool malloc = true) {
    auto vnormals = mesh_.get_vertex_property<opengp::Vec3>("v:normal");
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
    if (malloc) {
      glBufferData(GL_ARRAY_BUFFER, mesh_.n_vertices() * sizeof(opengp::Vec3), vnormals.data(), GL_STATIC_DRAW);
    } else {
      GLfloat *data = (GLfloat *) glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
      float *d  = (float *) vnormals.data();
      #pragma omp parallel for
      for (int i = 0; i < static_cast<int>(mesh_.n_vertices() * 3); ++i) {
        data[i] = d[i];
      }
      glUnmapBuffer(GL_ARRAY_BUFFER);
    }
  }
  void init(bool update = true) {
    if (update) mesh_.update_vertex_normals();
    /// Vertex Array
    glGenVertexArrays(1, &vertexArrayID_);
    glBindVertexArray(vertexArrayID_);

    /// Vertex Buffer
    glGenBuffers(1, &vertexBuffer_);
    updateVertexBuffer();
    /// Vertex Attribute ID
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    /// Normal Buffer
    glGenBuffers(1, &normalBuffer_);
    updateNormalBuffer();
    /// Vertex Attribute ID
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    /// Index Buffer
    std::vector<unsigned int> indices;
    for (auto fit = mesh_.faces_begin(); fit != mesh_.faces_end(); ++fit) {
      unsigned int n = mesh_.valence(*fit);
      auto vit = mesh_.vertices(*fit);
      for (unsigned int v = 0; v < n; ++v) {
        indices.push_back((*vit).idx());
        ++vit;
      }
    }
    numIndices_ = static_cast<int>(indices.size());
    GLuint triangleBuffer;
    glGenBuffers(1, &triangleBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triangleBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
  }
  opengp::Surface_mesh mesh_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A renderable Fullscreenquad.*/
class FullScreenQuad {
 public:
  FullScreenQuad() {
    init();
  }
  void display() {
    glBindVertexArray(vertexArrayID_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
  }
 private:
  void init() {
    /// Vertex Array
    glGenVertexArrays(1, &vertexArrayID_);
    glBindVertexArray(vertexArrayID_);
    /// Vertex Buffer
    GLfloat quad[] = {
      -1.0f, -1.0f, 0.0f,
      1.0f, -1.0f, 0.0f,
      -1.0f,  1.0f, 0.0f,
      -1.0f,  1.0f, 0.0f,
      1.0f, -1.0f, 0.0f,
      1.0f,  1.0f, 0.0f,
    };
    GLuint vertexBuffer;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad) , quad, GL_STATIC_DRAW);
    /// Vertex Attribute ID
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  }
  GLuint vertexArrayID_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief A SurfaceMesh with additional capabilities to use ShapeOp.*/
class ShapeOpSurfaceMesh {
 public:
  ShapeOpSurfaceMesh(const std::shared_ptr<GLSurfaceMeshObject> &mesh,
                     ShapeOp::Scalar bendind = 10.0,
                     ShapeOp::Scalar stretching = 10.0,
                     ShapeOp::Scalar closeness = 10.0) : mesh_(mesh) {
    init(closeness, stretching, bendind);
  }
  void init(ShapeOp::Scalar closeness,
            ShapeOp::Scalar stretching,
            ShapeOp::Scalar bendind) {
    opengp::Surface_mesh &m = mesh_->getMesh();
    auto vpoints = m.get_vertex_property<opengp::Vec3>("v:point");
    Eigen::Map<Eigen::Matrix3Xf> vertice((float *)(vpoints.data()), 3, m.n_vertices());
    ShapeOp::Matrix3X p = vertice.cast<ShapeOp::Scalar>();
    solver_.setPoints(p);
    addCloseness(closeness);
    addStretching(stretching);
    addBending(bendind);
//    solver_.addForces(std::make_shared<ShapeOp::GravityForce>(ShapeOp::Vector3(0.0, -10.0, 0.0)));
    fId_ = solver_.addForces(std::make_shared<ShapeOp::VertexForce>(ShapeOp::Vector3(0.0, 0.0, 1000.0), -1));
    solver_.initialize(true, 1.0, 1.0, 0.1);
  }
  void process() {
//    double t1 = glfwGetTime();
    solver_.solve(1);
//    std::cout << "solve: " << (glfwGetTime() - t1) * 1000.0 << " ms" << std::endl;
    opengp::Surface_mesh &m = mesh_->getMesh();
    auto vpoints = m.get_vertex_property<opengp::Vec3>("v:point");
    Eigen::Map<Eigen::Matrix3Xf> vertice((float *)(vpoints.data()), 3, m.n_vertices());
    const ShapeOp::Matrix3X &p = solver_.getPoints();
    vertice = p.cast<float>();
    mesh_->update();
//    std::cout << "animation: " << (glfwGetTime() - t1) * 1000.0 << " ms" << std::endl;
  }
  void enableForce(int id) {
    auto c = std::dynamic_pointer_cast<ShapeOp::VertexForce>(solver_.getForce(fId_)); //TODO: this will need to be robustify
    c->setId(id);
  }
  void rotate(ShapeOp::Scalar a) {
    Eigen::AngleAxis<ShapeOp::Scalar> aa(a, ShapeOp::Vector3::UnitY());
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(mesh_->getMesh().n_vertices()); ++i) {
      auto c = std::dynamic_pointer_cast<ShapeOp::ClosenessConstraint>(solver_.getConstraint(i));
      ShapeOp::Vector3 p = aa * c->getPosition();
      c->setPosition(p);
    }
  }

 private:
  void addStretching(ShapeOp::Scalar stretching) {
    opengp::Surface_mesh &m = mesh_->getMesh();
    auto f = std::bind(&ShapeOpSurfaceMesh::strainConstraint, this, stretching, std::placeholders::_1);
    edgeFuntional(m, f);
  }
  void strainConstraint(ShapeOp::Scalar weight, const std::vector<int> &id) {
    auto c = std::make_shared<ShapeOp::EdgeStrainConstraint>(id, weight, solver_.getPoints());
    solver_.addConstraint(c);
  }
  void addCloseness(ShapeOp::Scalar closeness) {
    opengp::Surface_mesh &m = mesh_->getMesh();
    auto vertices = m.vertex_property<opengp::Vec3>("v:point");
    Eigen::Map<Eigen::Matrix3Xf> positions = (Eigen::Map<Eigen::Matrix3Xf>((float *)(vertices.data()), 3, m.n_vertices()));
    for (int i = 0; i < positions.cols(); ++i) {
      std::vector<int> idI;
      idI.push_back(i);
      auto handles = std::make_shared<ShapeOp::ClosenessConstraint>(idI, closeness, solver_.getPoints());
      solver_.addConstraint(handles);
    }

  }
  void addBending(ShapeOp::Scalar bending) {
    opengp::Surface_mesh &m = mesh_->getMesh();
    auto f = std::bind(&ShapeOpSurfaceMesh::bendingConstraint, this, bending, std::placeholders::_1);
    trianglePairFuntional(m, f);
  }
  void bendingConstraint(ShapeOp::Scalar weight, const std::vector<int> &id) {
    auto c = std::make_shared<ShapeOp::BendingConstraint>(id, weight, solver_.getPoints());
    solver_.addConstraint(c);
  }
  std::shared_ptr<GLSurfaceMeshObject> mesh_;
  ShapeOp::Solver solver_;
  int fId_;
};
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
