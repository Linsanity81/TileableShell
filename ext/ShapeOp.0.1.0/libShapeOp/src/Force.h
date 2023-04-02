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
#ifndef FORCE_H
#define FORCE_H
///////////////////////////////////////////////////////////////////////////////
#include "Types.h"
///////////////////////////////////////////////////////////////////////////////
/** \file
This file contains all the forces (in fact defined as accelerations) of the ShapeOp library.*/
///////////////////////////////////////////////////////////////////////////////
namespace ShapeOp {
///////////////////////////////////////////////////////////////////////////////
/** \brief Base class of any forces. This class defines interface of a ShapeOp force.*/
class SHAPEOP_API Force {
 public:
  virtual ~Force() {;}
  /** \brief Get force vector.*/
  virtual Vector3 get(const Matrix3X &positions, int id) const = 0;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief This class defines a constant force for all vertices.*/
class SHAPEOP_API GravityForce : public Force {
 public:
  /** \brief Constructor taking the gravity vector as parameter.*/
  GravityForce(const Vector3 &f);
  virtual ~GravityForce() {;}
  /** \brief Get gravity vector.*/
  virtual Vector3 get(const Matrix3X &/*positions*/, int /*id*/) const override final;
 private:
  Vector3 f_;
};
///////////////////////////////////////////////////////////////////////////////
/** \brief This class defines a constant force for a unique vertex.*/
class SHAPEOP_API VertexForce : public Force {
 public:
  /** \brief Constructor taking the force and the vertex id as parameters.*/
  VertexForce(const Vector3 &f = Vector3::Zero(), int id = -1);
  virtual ~VertexForce() {;}
  /** \brief Get force vector.*/
  virtual Vector3 get(const Matrix3X &/*position*/, int id) const override final;
  /** \brief Set a new vertex id.*/
  void setId(int id);
  /** \brief Set a new force.*/
  void setForce(const Vector3 &f);
 private:
  Vector3 f_;
  int id_;
};
///////////////////////////////////////////////////////////////////////////////
} // namespace ShapeOp
///////////////////////////////////////////////////////////////////////////////
#ifdef SHAPEOP_HEADER_ONLY
#include "Force.cpp"
#endif
///////////////////////////////////////////////////////////////////////////////
#endif // FORCE_H
///////////////////////////////////////////////////////////////////////////////
