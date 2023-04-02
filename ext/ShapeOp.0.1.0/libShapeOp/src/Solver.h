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
#ifndef SOLVER_H
#define SOLVER_H
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include <vector>
#include <memory>
#include "Types.h"
///////////////////////////////////////////////////////////////////////////////
/** @file
This file contains the main ShapeOp solver.*/
///////////////////////////////////////////////////////////////////////////////
namespace ShapeOp {
///////////////////////////////////////////////////////////////////////////////
// Forward Declarations
class LSSolver;
class Constraint;
class Force;
///////////////////////////////////////////////////////////////////////////////
/** \brief ShapeOp Solver. This class implements the main ShapeOp solver based on \cite Bouaziz2012 and \cite Bouaziz2014.*/
class SHAPEOP_API Solver {
 public:
  /** \brief Add a constraint to the solver and get back its id.*/
  int addConstraint(const std::shared_ptr<Constraint> &c);
  /** \brief Get a constraint using its id.*/
  std::shared_ptr<Constraint> &getConstraint(int id);
  /** \brief Add a force to the solver and get back its id.*/
  int addForces(const std::shared_ptr<Force> &f);
  /** \brief Get a force using its id.*/
  std::shared_ptr<Force> &getForce(int id);
  /** \brief Set the points.*/
  void setPoints(const Matrix3X &p);
  /** \brief Set the timestep for the dynamics.*/
  void setTimeStep(Scalar timestep);
  /** \brief Set the velocity damping for the dynamics.*/
  void setDamping(Scalar damping);
  /** \brief Get the points.*/
  const Matrix3X &getPoints();
  /** \brief Initialize the ShapeOp linear system and the different parameters.
  \return true if successfull */
  bool initialize(bool dynamic = false, Scalar masses = 1.0, Scalar damping = 1.0, Scalar timestep = 1.0);
  /** \brief Solve the constraint problem by projecting and merging.
    \return true if successfull */
  bool solve(unsigned int iteration);
 private:
  typedef std::vector<std::shared_ptr<Constraint> > Constraints;
  typedef std::vector<std::shared_ptr<Force> > Forces;

//Static
  Matrix3X points_;
  Matrix3X projections_;
  Constraints constraints_;
  std::shared_ptr<LSSolver> solver_;
  SparseMatrix At_;
  SparseMatrix N_;

//Dynamic
  bool dynamic_;
  SparseMatrix M_;
  Matrix3X oldPoints_;
  Matrix3X velocities_;
  Matrix3X momentum_;
  Scalar masses_;
  Forces forces_;
  Scalar damping_;
  Scalar delta_;
};
///////////////////////////////////////////////////////////////////////////////
} // namespace ShapeOp
///////////////////////////////////////////////////////////////////////////////
#ifdef SHAPEOP_HEADER_ONLY
#include "Solver.cpp"
#endif
///////////////////////////////////////////////////////////////////////////////
#endif // SOLVER_H
///////////////////////////////////////////////////////////////////////////////
