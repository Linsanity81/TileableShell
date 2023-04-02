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
#ifndef CAMERA_H
#define CAMERA_H
///////////////////////////////////////////////////////////////////////////////
#include <Types.h>
///////////////////////////////////////////////////////////////////////////////
/** \brief Camera. A Camera based on Eigen.*/
class Camera {
 public:
  /** \brief Camera Constructor. */
  Camera(const Eigen::Vector3f &position, const Eigen::Vector3f &aim, float aspectRatio = 1.0) :
    position_(position),
    aim_(aim),
    up_(0.0f, 1.0f, 0.0f),
    aspectRatio_(aspectRatio),
    fieldOfView_(45.0f),
    nearPlane_(0.1f),
    farPlane_(100.0f) {
    computeDirection();
    updateProjection();
    updateView();
  }

  /** \brief Updates the projection matrix of the camera and its inverse projection.*/
  void updateProjection() const {
    projection_ = perspective(aspectRatio_, fieldOfView_, nearPlane_, farPlane_);
    projectionInverse_ = projection_.inverse();
  }

  /** \brief Updates the view matrix of the camera and its inverse view matrix.*/
  void updateView() const {
    view_ = lookAt(position_, aim_, up_);
    viewInverse_ = view_.inverse();
  }
  /** \brief Get the cameras projection matrix.*/
  const Eigen::Matrix4f &getProjection() const {
    return projection_;
  }
  /** \brief Get the cameras inverse projection matrix.*/
  const Eigen::Matrix4f &getInverseProjection() const {
    return projectionInverse_;
  }
  /** \brief Get the cameras view matrix.*/
  const Eigen::Matrix4f &getView() const {
    return view_;
  }
  /** \brief Get the cameras inverse view matrix.*/
  const Eigen::Matrix4f &getInverseView() const {
    return viewInverse_;
  }

  void rotateStraff(float angle) {
    Eigen::Vector3f straf = (direction_.cross(up_)).normalized();
    rotate(angle, straf);
  }

  void rotateUp(float angle) {
    rotate(angle, up_);
  }
  void move(float speed) {
    position_ += (direction_ * speed);
    aim_ += (direction_ * speed);
    computeDirection();
  }
  void straff(float speed) {
    Eigen::Vector3f straf = (direction_.cross(up_)).normalized();
    position_ += (straf * speed);
    aim_ += (straf * speed);
    computeDirection();
  }
  void flight(float speed) {
    position_ += (up_ * speed);
    aim_ += (up_ * speed);
    computeDirection();
  }
  void setPosition(const Eigen::Vector3f &position) {
    position_ = position;
  }
  void setAim(const Eigen::Vector3f &aim) {
    aim_ = aim;
  }
  const Eigen::Vector3f &getPosition() const {
    return position_ ;
  }
  const Eigen::Vector3f &getAim() const {
    return aim_;
  }
 private:
  void rotate(float angle, const Eigen::Vector3f axis) {
    direction_ = Eigen::AngleAxisf(angle, axis) * direction_;
    aim_ = position_ + direction_;
  }
  void computeDirection() {
    direction_ = aim_ - position_;
    direction_.normalize();
  }
  Eigen::Matrix4f lookAt(const Eigen::Vector3f &eye, const Eigen::Vector3f &center, const Eigen::Vector3f &up) const {
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f u = up.normalized();
    Eigen::Vector3f s = f.cross(u).normalized();
    u = s.cross(f);
    Eigen::Matrix4f mat = Eigen::Matrix4f::Zero();
    mat(0, 0) = s.x();
    mat(0, 1) = s.y();
    mat(0, 2) = s.z();
    mat(0, 3) = -s.dot(eye);
    mat(1, 0) = u.x();
    mat(1, 1) = u.y();
    mat(1, 2) = u.z();
    mat(1, 3) = -u.dot(eye);
    mat(2, 0) = -f.x();
    mat(2, 1) = -f.y();
    mat(2, 2) = -f.z();
    mat(2, 3) = f.dot(eye);
    mat.row(3) << 0, 0, 0, 1;
    return mat;
  }
  Eigen::Matrix4f perspective(float aspect, float fovy, float zNear, float zFar) const {
    Eigen::Transform<float, 3, Eigen::Projective> tr;
    tr.matrix().setZero();
    float radf = M_PI * fovy / 180.0;
    float tan_half_fovy = std::tan(radf / 2.0);
    tr(0, 0) = 1.0 / (aspect * tan_half_fovy);
    tr(1, 1) = 1.0 / (tan_half_fovy);
    tr(2, 2) = - (zFar + zNear) / (zFar - zNear);
    tr(3, 2) = - 1.0;
    tr(2, 3) = - (2.0 * zFar * zNear) / (zFar - zNear);
    return tr.matrix();
  }
  Eigen::Vector3f position_;
  Eigen::Vector3f aim_;
  Eigen::Vector3f up_;
  Eigen::Vector3f direction_;
  float aspectRatio_;
  float fieldOfView_;
  float nearPlane_;
  float farPlane_;
  mutable Eigen::Matrix4f view_;
  mutable Eigen::Matrix4f viewInverse_;
  mutable Eigen::Matrix4f projection_;
  mutable Eigen::Matrix4f projectionInverse_;
};
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
