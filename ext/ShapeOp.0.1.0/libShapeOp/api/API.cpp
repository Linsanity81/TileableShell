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
#include "API.h"
#include "Solver.h"
#include "Constraint.h"
#include "Force.h"
///////////////////////////////////////////////////////////////////////////////
/** \brief C structure that containts the C++ ShapeOp solver.*/
struct ShapeOpSolver {
  /** \brief A shared pointer to a C++ ShapeOp solver.*/
  std::shared_ptr<ShapeOp::Solver> s;
};
///////////////////////////////////////////////////////////////////////////////
extern ShapeOpSolver *shapeop_create() {
  ShapeOpSolver *solver = new ShapeOpSolver;
  solver->s = std::make_shared<ShapeOp::Solver>();
  return solver;
}
extern void shapeop_delete(ShapeOpSolver *op) {
  delete op;
}
extern int shapeop_init(ShapeOpSolver *op) {
  return static_cast<int>(!op->s->initialize());
}
extern int  shapeop_initDynamic(ShapeOpSolver *op,  ShapeOpScalar masses, ShapeOpScalar damping, ShapeOpScalar timestep) {
  return static_cast<int>(!op->s->initialize(true, masses, damping, timestep));
}
extern int shapeop_solve(ShapeOpSolver *op, unsigned int iteration) {
  return static_cast<int>(!op->s->solve(iteration));
}
extern void shapeop_setPoints(ShapeOpSolver *op, ShapeOpScalar *points, int nb_points) {
  Eigen::Map<ShapeOp::Matrix3X> p(points, 3, nb_points);
  op->s->setPoints(p);
}
extern void shapeop_getPoints(ShapeOpSolver *op, ShapeOpScalar *points, int nb_points) {
  Eigen::Map<ShapeOp::Matrix3X> p(points, 3, nb_points);
  p = op->s->getPoints();
}
extern void shapeop_setTimeStep(ShapeOpSolver *op, ShapeOpScalar timestep) {
  op->s->setTimeStep(timestep);
}
extern void shapeop_setDamping(ShapeOpSolver *op, ShapeOpScalar damping) {
  op->s->setDamping(damping);
}
///////////////////////////////////////////////////////////////////////////////
extern int shapeop_addConstraint(ShapeOpSolver *op, const char *constraintType, int *ids, int nb_ids, ShapeOpScalar weight) {

  const std::string ct(constraintType);
  std::vector<int> idv(ids, ids + nb_ids);
  std::shared_ptr<ShapeOp::Constraint> c = ShapeOp::Constraint::shapeConstraintFactory(ct, idv, weight, op->s->getPoints());
  if (!c) { return -1; }
  return op->s->addConstraint(c);
}
extern shapeop_err shapeop_editConstraint(ShapeOpSolver *op,
                                          const char *constraintType,
                                          int constraint_id,
                                          const ShapeOpScalar *scalars,
                                          int nb_scl) {

  if (strcmp(constraintType, "EdgeStrain") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::EdgeStrainConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 3) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setEdgeLength(scalars[0]);
    c->setRangeMin(scalars[1]);
    c->setRangeMax(scalars[2]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "TriangleStrain") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::TriangleStrainConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setRangeMin(scalars[0]);
    c->setRangeMax(scalars[1]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "TetrahedronStrain") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::TetrahedronStrainConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setRangeMin(scalars[0]);
    c->setRangeMax(scalars[1]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Area") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::AreaConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setRangeMin(scalars[0]);
    c->setRangeMax(scalars[1]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Volume") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::VolumeConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setRangeMin(scalars[0]);
    c->setRangeMax(scalars[1]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Bending") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::BendingConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setRangeMin(scalars[0]);
    c->setRangeMax(scalars[1]);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Closeness") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::ClosenessConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 3) { return SO_INVALID_ARGUMENT_LENGTH; }
    Eigen::Map<const ShapeOp::Vector3> p(scalars, 3, 1);
    c->setPosition(p);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Similarity") == 0 || strcmp(constraintType, "Rigid") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::SimilarityConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    int nI = static_cast<int>(c->nIndices());
    if ((nb_scl % (nI * 3)) != 0) { return SO_INVALID_ARGUMENT_LENGTH; }
    std::vector<ShapeOp::Matrix3X> shapes;
    int nShapes = nb_scl / (nI * 3);
    for (int i = 0; i < nShapes; ++i) {
      Eigen::Map<const ShapeOp::Matrix3X> s(scalars + i * nI * 3, 3, nI);
      shapes.push_back(s);
    }
    c->setShapes(shapes);
    return SO_SUCCESS;
  }
  if (strcmp(constraintType, "Angle") == 0) {
    auto c = std::dynamic_pointer_cast<ShapeOp::AngleConstraint>(op->s->getConstraint(constraint_id));
    if (!c) { return SO_UNMATCHING_CONSTRAINT_ID; }
    if (nb_scl != 2) { return SO_INVALID_ARGUMENT_LENGTH; }
    c->setMinAngle(scalars[0]);
    c->setMaxAngle(scalars[1]);
    return SO_SUCCESS;
  }
  return SO_INVALID_CONSTRAINT_TYPE;
}
extern int shapeop_addUniformLaplacianConstraint(ShapeOpSolver *op, int *ids, int nb_ids,
                                                 int displacement_lap, ShapeOpScalar weight) {
  std::vector<int> id_vector(ids, ids + nb_ids);
  auto c = std::make_shared<ShapeOp::UniformLaplacianConstraint>(id_vector, weight, op->s->getPoints(), displacement_lap != 0);
  return op->s->addConstraint(c);
}
///////////////////////////////////////////////////////////////////////////////
extern int shapeop_addGravityForce(ShapeOpSolver *op, ShapeOpScalar *force) {
  Eigen::Map<ShapeOp::Vector3> g(force, 3, 1);
  auto f = std::make_shared<ShapeOp::GravityForce>(g);
  return op->s->addForces(f);
}
extern int shapeop_addVertexForce(ShapeOpSolver *op, ShapeOpScalar *force, int id) {
  Eigen::Map<ShapeOp::Vector3> g(force, 3, 1);
  auto f = std::make_shared<ShapeOp::VertexForce>(g, id);
  return op->s->addForces(f);
}
extern void shapeop_editVertexForce(ShapeOpSolver *op, int force_id, ShapeOpScalar *force, int id) {
  Eigen::Map<ShapeOp::Vector3> g(force, 3, 1);
  auto f = std::dynamic_pointer_cast<ShapeOp::VertexForce>(op->s->getForce(force_id)); //TODO: this will need to be robustify
  f->setId(id);
  f->setForce(g);
}
///////////////////////////////////////////////////////////////////////////////
