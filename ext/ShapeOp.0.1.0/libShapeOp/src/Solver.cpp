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
#include "Solver.h"
#include "LSSolver.h"
#include "Constraint.h"
#include "Force.h"
///////////////////////////////////////////////////////////////////////////////
#ifdef SHAPEOP_OPENMP
#ifdef SHAPEOP_MSVC
#define SHAPEOP_OMP_PARALLEL __pragma(omp parallel)
#define SHAPEOP_OMP_FOR __pragma(omp for)
#else
#define SHAPEOP_OMP_PARALLEL _Pragma("omp parallel")
#define SHAPEOP_OMP_FOR _Pragma("omp for")
#endif
#else
#define SHAPEOP_OMP_PARALLEL
#define SHAPEOP_OMP_FOR
#endif
///////////////////////////////////////////////////////////////////////////////
namespace ShapeOp {
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE int Solver::addConstraint(const std::shared_ptr<Constraint> &c) {
  constraints_.push_back(c);
  return static_cast<int>(constraints_.size() - 1);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE std::shared_ptr<Constraint> &Solver::getConstraint(int id) {
  return constraints_[id];
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE int Solver::addForces(const std::shared_ptr<Force> &f) {
  forces_.push_back(f);
  return static_cast<int>(forces_.size() - 1);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE std::shared_ptr<Force> &Solver::getForce(int id) {
  return forces_[id];
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void Solver::setPoints(const Matrix3X &p) {
  points_ = p;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void Solver::setTimeStep(Scalar timestep) {
  delta_ = timestep;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void Solver::setDamping(Scalar damping) {
  damping_ = damping;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE const Matrix3X &Solver::getPoints() {
  return points_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE bool Solver::initialize(bool dynamic, Scalar masses, Scalar damping, Scalar timestep) {
  int n_points = static_cast<int>(points_.cols());
  int n_constraints = static_cast<int>(constraints_.size());
  assert(n_points != 0);
  assert(n_constraints != 0);
  std::vector<Triplet> triplets;
  int idO = 0;
  for (int i = 0; i < n_constraints; ++i) constraints_[i]->addConstraint(triplets, idO);
  projections_.setZero(3, idO);
  SparseMatrix A = SparseMatrix(idO, n_points);
  A.setFromTriplets(triplets.begin(), triplets.end());
  At_ = A.transpose();
  oldPoints_ = Matrix3X(3, n_points);
  //Dynamic
  velocities_ = Matrix3X::Zero(3, n_points);
  momentum_ = Matrix3X::Zero(3, n_points);
  M_ = SparseMatrix(n_points, n_points);
  masses_ = masses;
  damping_ = damping;
  delta_ = timestep;
  dynamic_ = dynamic;
  M_.setIdentity();
  M_ *= masses_; //TODO: fix this
  M_ /= delta_ * delta_;
  N_ = At_ * A;
  if (dynamic_) N_ += M_;
  solver_ = std::make_shared<ShapeOp::SimplicialLDLTSolver>();
  solver_->initialize(N_);
  return solver_->info() == Eigen::Success;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE bool Solver::solve(unsigned int iteration) {
  SHAPEOP_OMP_PARALLEL
  {
    //momentum
    if (dynamic_) {
      SHAPEOP_OMP_FOR for (int i = 0; i < static_cast<int>(momentum_.cols()); ++i) {
        momentum_.col(i) = points_.col(i) + delta_ * velocities_.col(i);
        for (int j = 0; j < static_cast<int>(forces_.size()); ++j) momentum_.col(i) += (delta_ * delta_) * forces_[j]->get(points_, i); //TODO: add more general external forces
        oldPoints_.col(i) = points_.col(i);
        points_.col(i) = momentum_.col(i);
      }
    } else {
      SHAPEOP_OMP_FOR for (int i = 0; i < static_cast<int>(momentum_.cols()); ++i) {
        momentum_.col(i).setZero();
        for (int j = 0; j < static_cast<int>(forces_.size()); ++j) momentum_.col(i) += (delta_ * delta_) * forces_[j]->get(points_, i); //TODO: add more general external forces
      }
    }
  }
  for (unsigned int it = 0; it < iteration; ++it) {
    SHAPEOP_OMP_PARALLEL
    {
      //local solve: projection
      SHAPEOP_OMP_FOR for (int i = 0; i < static_cast<int>(constraints_.size()); ++i)
        constraints_[i]->project(points_, projections_);
      //global solve:  merging
      SHAPEOP_OMP_FOR for (int i = 0; i < 3; ++i)
        points_.row(i) = solver_->solve(At_ * projections_.row(i).transpose() + M_ * momentum_.row(i).transpose(), points_.row(i).transpose()).transpose(); //TODO: should At_ nd M_ be row major? Temporary variables?
    }
  }
  if (dynamic_) {
    SHAPEOP_OMP_PARALLEL
    {
      //velocity update
      SHAPEOP_OMP_FOR for (int i = 0; i < static_cast<int>(velocities_.cols()); ++i) {
        velocities_.col(i) = damping_ * (points_.col(i) - oldPoints_.col(i)) / delta_;
      }
    }
  }
  return solver_->info() == Eigen::Success;
}
///////////////////////////////////////////////////////////////////////////////
} // namespace ShapeOp
///////////////////////////////////////////////////////////////////////////////
