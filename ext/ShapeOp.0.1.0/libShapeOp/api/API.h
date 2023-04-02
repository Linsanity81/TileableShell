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
#ifndef API_H
#define API_H
///////////////////////////////////////////////////////////////////////////////
#include "Common.h"
///////////////////////////////////////////////////////////////////////////////
/** \file
* This file implements a C API for the ShapeOp C++ library.
*
* To use the library you need to:
*
* 1) Create the solver with #shapeop_create
*
* 2) Set the vertices with #shapeop_setPoints
*
* 3) Setup the constraints and forces with #shapeop_addConstraint and #shapeop_editConstraint
*
* 4) Initalize the solver with #shapeop_init or #shapeop_initDynamic
*
* 5) Optimize with #shapeop_solve
*
* 6) Get back the vertices with #shapeop_getPoints
*
* 7) Delete the solver with #shapeop_delete
*/
///////////////////////////////////////////////////////////////////////////////
/** \brief C structure that containts the C++ #ShapeOp::Solver.*/
typedef struct ShapeOpSolver ShapeOpSolver;
///////////////////////////////////////////////////////////////////////////////
/** \brief ShapeOp Success and Error type. This list might be extended. To simply test for errors, use =! SO_SUCCESS.*/
typedef enum shapeop_err {
  SO_SUCCESS = 0,          /** \brief ShapeOp Success type indicating that no error happened.*/
  SO_INVALID_CONSTRAINT_TYPE = 1, /** \brief ShapeOp Error type indicating an invalid constraint type provided.*/
  SO_INVALID_ARGUMENT_LENGTH = 2, /** \brief ShapeOp Error type indicating an invalid length of an array argument.*/
  SO_UNMATCHING_CONSTRAINT_ID = 3/** \brief ShapeOp Error type indicating that the constraint type does not match the id provided.*/
} shapeop_err;
///////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
///////////////////////////////////////////////////////////////////////////////
// Solver
/** \brief Create the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API ShapeOpSolver *shapeop_create();
/** \brief Delete the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API void shapeop_delete(ShapeOpSolver *op);
/** \brief Initialize the ShapeOp solver for static geometry processing. For more details see #ShapeOp::Solver.
  \return 0 on success, 1 otherwise */
SHAPEOP_API int  shapeop_init(ShapeOpSolver *op);
/** \brief Initialize the ShapeOp solver for dynamic geometry processing. For more details see #ShapeOp::Solver.
  \return 0 on success, 1 otherwise */
SHAPEOP_API int  shapeop_initDynamic(ShapeOpSolver *op,  ShapeOpScalar masses, ShapeOpScalar damping, ShapeOpScalar timestep);
/** \brief Run the optimization. For more details see #ShapeOp::Solver.
  \return 0 on success, 1 otherwise */
SHAPEOP_API int  shapeop_solve(ShapeOpSolver *op, unsigned int iteration);

/** \brief Set the vertices to the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API void shapeop_setPoints(ShapeOpSolver *op, ShapeOpScalar *points, int nb_points);
/** \brief Get the vertices back from the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API void shapeop_getPoints(ShapeOpSolver *op, ShapeOpScalar *points, int nb_points);

/** \brief Set the timestep of the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API void shapeop_setTimeStep(ShapeOpSolver *op, ShapeOpScalar timestep);
/** \brief Set the damping of the ShapeOp solver. For more details see #ShapeOp::Solver.*/
SHAPEOP_API void shapeop_setDamping(ShapeOpSolver *op, ShapeOpScalar damping);

///////////////////////////////////////////////////////////////////////////////
// Constraints

/** \brief Add a constraint to the ShapeOp solver.
  \param op The ShapeOp Solver object
  \param constraintType A c-style string containing one of the following constraint types listed in the documentation of #ShapeOp::Constraint::shapeConstraintFactory:
     - "EdgeStrain", takes 2 indices forming an edge. See #ShapeOp::EdgeStrainConstraint
     - "TriangleStrain", takes 3 indices forming a triangle. See #ShapeOp::TriangleStrainConstraint
     - "TetrahedronStrain", takes 4 indices forming a tetrahedron. See #ShapeOp::TetrahedronStrainConstraint
     - "Area", takes 3 indices forming a triangle. See #ShapeOp::AreaConstraint
     - "Volume", takes 4 indices forming a tetrahedron. See #ShapeOp::VolumeConstraint
     - "Bending", takes 4 indices of two neighboring triangles sharing an edge. See #ShapeOp::BendingConstraint
     - "Closeness", takes 1 index. See #ShapeOp::ClosenessConstraint
     - "Line", takes 2 or more indices. See #ShapeOp::LineConstraint
     - "Plane", takes 3 or more indices. See #ShapeOp::PlaneConstraint
     - "Circle", takes 3 or more indices. See #ShapeOp::CircleConstraint
     - "Sphere", takes 4 or more indices. See #ShapeOp::SphereConstraint
     - "Similarity", takes 1 or more indices. See #ShapeOp::SimilarityConstraint with scaling
     - "Rigid", takes 1 or more indices. See #ShapeOp::SimilarityConstraint without scaling
     - "Rectangle", takes 4 indices. See #ShapeOp::RectangleConstraint
     - "Parallelogram", takes 4 indices. See #ShapeOp::ParallelogramConstraint
     - "Laplacian", takes 2 or more indices, center vertex first, then the one ring neighborhood. See #ShapeOp::UniformLaplacianConstraint
     - "LaplacianDisplacement", takes 2 or more indices, center vertex first, then the one ring neighborhood. See #ShapeOp::UniformLaplacianConstraint of deformation.
     - "Angle", takes 3 indices forming two consecutive edges. See #ShapeOp::AngleConstraint.
  \param ids The array of indices of the points subject to the constraint. See documentation of the corresponding constraint in Constraint.h
  \param nb_ids The length or number of indices in ids.
  \param weight The weight of the constraint to be added relative to the other constraints in the ShapeOpSolver.
  \return constraint index in the solver. -1 if adding failed.
*/
SHAPEOP_API int shapeop_addConstraint(ShapeOpSolver *op, const char *constraintType, int *ids, int nb_ids, ShapeOpScalar weight);

/** \brief Add a constraint to the ShapeOp solver.
  \param op The ShapeOp Solver object
  \param constraintType A c-style string containing one of the following constraint types
     - "EdgeStrain", see #ShapeOp::EdgeStrainConstraint. The scalars have 3 entries: (1) the desired distance between the two constrained vertices; (2) rangeMin; (3) rangeMax.
     - "TriangleStrain", see #ShapeOp::TriangleStrainConstraint. The scalars have 2 entries: (1) rangeMin; (2) rangeMax.
     - "TetrahedronStrain", see #ShapeOp::TetrahedronStrainConstraint. The scalars have 2 entries: (1) rangeMin; (2) rangeMax.
     - "Area", see #ShapeOp::AreaConstraint. The scalars have 2 entries: (1) rangeMin; (2) rangeMax.
     - "Volume", see #ShapeOp::VolumeConstraint. The scalars have 2 entries: (1) rangeMin; (2) rangeMax.
     - "Bending", see #ShapeOp::BendingConstraint. The scalars have 2 entries: (1) rangeMin; (2) rangeMax.
     - "Closeness", see #ShapeOp::ClosenessConstraint. The scalars have 3 entries, which are the desire coordinates of constrained vertex.
     - "Similarity", see #ShapeOp::SimilarityConstraint with scaling. The scalars have 3 * n * m entries, where m is the number of candidate shapes that the constrained vertices should be similar to,
        and n is the number of points in each candidate shape (the same as the number of constrained vertices). Each block of 3 * n scalars provides the point coordinates of a candidate shape.
     - "Rigid", see #ShapeOp::SimilarityConstraint without scaling. Same as for "Similarity", see above.
     - "Angle", see #ShapeOp::AngleConstraint. The scalars have 2 entries: (1) minAngle; (2) maxAngle.
  \param constraint_id The id of the constraint, which is returned in #shapeop_addConstraint.
  \param scalars A c-style array of #ShapeOpScalar's. See documentation of the corresponding constraint in Constraint.h
  \param nb_scl The length of the array scalars.
  \return See #shapeop_err
*/
SHAPEOP_API shapeop_err shapeop_editConstraint(ShapeOpSolver *op,
                                               const char *constraintType,
                                               int constraint_id,
                                               const ShapeOpScalar *scalars,
                                               int nb_scl);
///////////////////////////////////////////////////////////////////////////////
// Forces
/** \brief Add a gravity force to the ShapeOp solver. For more details see #ShapeOp::GravityForce.
  \param op The ShapeOp Solver object
  \param force A c-style array of 3 #ShapeOpScalar's specifying the force vector in each dimension
*/
SHAPEOP_API int shapeop_addGravityForce(ShapeOpSolver *op, ShapeOpScalar *force);

/** \brief Add a vertex force to the ShapeOp solver. For more details see #ShapeOp::VertexForce.*/
SHAPEOP_API int shapeop_addVertexForce(ShapeOpSolver *op, ShapeOpScalar *force, int id);
/** \brief Edit a vertex force previously added to the ShapeOp solver. For more details see #ShapeOp::VertexForce.*/
SHAPEOP_API void shapeop_editVertexForce(ShapeOpSolver *op, int force_id, ShapeOpScalar *force, int id);
///////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif
///////////////////////////////////////////////////////////////////////////////
#endif // API_H
///////////////////////////////////////////////////////////////////////////////
