///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014-2015 Sofien Bouaziz <sofien.bouaziz@gmail.com>, Bailin Deng <bldeng@gmail.com>, Mario Deuss <mario.deuss@epfl.ch>
// Copyright (C) 2014-2015 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#include "Constraint.h"
#include <cassert>
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////
#define SHAPEOP_INNER_ITERATIONS 4 //TODO: fix this
///////////////////////////////////////////////////////////////////////////////
namespace ShapeOp {
///////////////////////////////////////////////////////////////////////////////
/** \brief Clamps v to lie between vMin and vMax.*/
SHAPEOP_INLINE Scalar clamp(Scalar v, Scalar vMin, Scalar vMax) {
  Scalar result = v > vMin ? v : vMin;
  return result > vMax ? vMax : result;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE std::shared_ptr<Constraint> Constraint::shapeConstraintFactory(const std::string &constraintType, const std::vector<int> &idI, Scalar weight, const Matrix3X &positions) {

  std::size_t n = idI.size();
  std::shared_ptr<Constraint> c;
  if (constraintType.compare("EdgeStrain") == 0) {        if (n != 2) { return c;} return std::make_shared<EdgeStrainConstraint>(idI, weight, positions); }
  if (constraintType.compare("TriangleStrain") == 0) {    if (n != 3) { return c; } return std::make_shared<TriangleStrainConstraint>(idI, weight, positions); }
  if (constraintType.compare("TetrahedronStrain") == 0) { if (n != 4) { return c; } return std::make_shared<TetrahedronStrainConstraint>(idI, weight, positions); }
  if (constraintType.compare("Area") == 0) {              if (n != 3) { return c; } return std::make_shared<AreaConstraint>(idI, weight, positions); }
  if (constraintType.compare("Volume") == 0) {            if (n != 4) { return c; } return std::make_shared<VolumeConstraint>(idI, weight, positions); }
  if (constraintType.compare("Bending") == 0) {           if (n != 4) { return c; } return std::make_shared<BendingConstraint>(idI, weight, positions); }
  if (constraintType.compare("Closeness") == 0) {         if (n != 1) { return c; } return std::make_shared<ClosenessConstraint>(idI, weight, positions); }
  if (constraintType.compare("Line") == 0) {              if (n <  2) { return c; } return std::make_shared<LineConstraint>(idI, weight, positions); }
  if (constraintType.compare("Plane") == 0) {             if (n <  3) { return c; } return std::make_shared<PlaneConstraint>(idI, weight, positions); }
  if (constraintType.compare("Circle") == 0) {            if (n <  3) { return c; } return std::make_shared<CircleConstraint>(idI, weight, positions); }
  if (constraintType.compare("Sphere") == 0) {            if (n <  4) { return c; } return std::make_shared<SphereConstraint>(idI, weight, positions); }
  if (constraintType.compare("Similarity") == 0) {        if (n <  1) { return c; } return std::make_shared<SimilarityConstraint>(idI, weight, positions, true); }
  if (constraintType.compare("Rigid") == 0) {             if (n <  1) { return c; } return std::make_shared<SimilarityConstraint>(idI, weight, positions, false); }
  if (constraintType.compare("Rectangle") == 0) {         if (n != 4) { return c; } return std::make_shared<RectangleConstraint>(idI, weight, positions); }
  if (constraintType.compare("Parallelogram") == 0) {     if (n != 4) { return c; } return std::make_shared<ParallelogramConstraint>(idI, weight, positions); }
  if (constraintType.compare("Laplacian") == 0) {         if (n <  2) { return c; } return std::make_shared<UniformLaplacianConstraint>(idI, weight, positions, false); }
  if (constraintType.compare("LaplacianDisplacement") == 0) {  if (n <  2) { return c; } return std::make_shared<UniformLaplacianConstraint>(idI, weight, positions, true); }
  if (constraintType.compare("Angle") == 0) {   if (n !=  3) { return c; } return std::make_shared<AngleConstraint>(idI, weight, positions); }
  return c;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE Constraint::Constraint(const std::vector<int> &idI, Scalar weight) :
  idI_(idI),
  weight_(std::sqrt(weight)) {
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE EdgeStrainConstraint::EdgeStrainConstraint(const std::vector<int> &idI,
                                                          Scalar weight,
                                                          const Matrix3X &positions,
                                                          Scalar rangeMin,
                                                          Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 2);
  Scalar length = (positions.col(idI_[1]) - positions.col(idI_[0])).norm();
  rest_ = 1.0f / length;
  weight_ *= std::sqrt(length);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void EdgeStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Vector3 edge = positions.col(idI_[1]) - positions.col(idI_[0]);
  Scalar l = edge.norm();
  edge /= l;
  l = clamp(l * rest_, rangeMin_, rangeMax_);
  projections.col(idO_) = weight_ * l * edge;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void EdgeStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  triplets.push_back(Triplet(idO_, idI_[0], -weight_ * rest_));
  triplets.push_back(Triplet(idO_, idI_[1], weight_ * rest_));
  idO += 1;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void EdgeStrainConstraint::setEdgeLength(Scalar length) {
  rest_ = 1.0f / length;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE TriangleStrainConstraint::TriangleStrainConstraint(const std::vector<int> &idI,
                                                                  Scalar weight,
                                                                  const Matrix3X &positions,
                                                                  Scalar rangeMin,
                                                                  Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 3);
  Matrix32 edges, P;
  edges.col(0) = positions.col(idI_[1]) - positions.col(idI_[0]);
  edges.col(1) = positions.col(idI_[2]) - positions.col(idI_[0]);
  P.col(0) = edges.col(0).normalized();
  P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
  rest_ = (P.transpose() * edges).inverse();
  Scalar A = (P.transpose() * edges).determinant() / 2.0f;
  weight_ *= std::sqrt(std::abs(A));
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void TriangleStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix32 edges, P;
  edges.col(0) = (positions.col(idI_[1]) - positions.col(idI_[0]));
  edges.col(1) = (positions.col(idI_[2]) - positions.col(idI_[0]));
  P.col(0) = edges.col(0).normalized();
  P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
  Matrix22 F = P.transpose() * edges * rest_;
  Eigen::JacobiSVD<Matrix22> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector2 S = svd.singularValues();
  S(0) = clamp(S(0), rangeMin_, rangeMax_);
  S(1) = clamp(S(1), rangeMin_, rangeMax_);
  F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
  projections.block<3, 2>(0, idO_) = (weight_ * P * F);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void TriangleStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 2;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
  }
  idO += n;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE TetrahedronStrainConstraint::TetrahedronStrainConstraint(const std::vector<int> &idI,
                                                                        Scalar weight,
                                                                        const Matrix3X &positions,
                                                                        Scalar rangeMin,
                                                                        Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 4);
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  rest_ = edges.inverse();
  Scalar V = (edges).determinant() / 6.0f;
  weight_ *= std::sqrt(std::abs(V));
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void TetrahedronStrainConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  Matrix33 F = edges * rest_;
  Eigen::JacobiSVD<Matrix33> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3 S = svd.singularValues();
  S(0) = clamp(S(0), rangeMin_, rangeMax_);
  S(1) = clamp(S(1), rangeMin_, rangeMax_);
  S(2) = clamp(S(2), rangeMin_, rangeMax_);
  if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0f) S(2) = -S(2);
  F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
  projections.block<3, 3>(0, idO_) = weight_ * F;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void TetrahedronStrainConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 3;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i) + rest_(2, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[3], weight_ * rest_(2, i)));
  }
  idO += n;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE AreaConstraint::AreaConstraint(const std::vector<int> &idI,
                                              Scalar weight,
                                              const Matrix3X &positions,
                                              Scalar rangeMin,
                                              Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI.size() == 3);
  Matrix32 edges, P;
  edges.col(0) = positions.col(idI_[1]) - positions.col(idI_[0]);
  edges.col(1) = positions.col(idI_[2]) - positions.col(idI_[0]);
  P.col(0) = edges.col(0).normalized();
  P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
  rest_ = (P.transpose() * edges).inverse();
  Scalar A = (P.transpose() * edges).determinant() / 2.0f;
  weight_ *= std::sqrt(std::abs(A));
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AreaConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix32 edges, P;
  edges.col(0) = (positions.col(idI_[1]) - positions.col(idI_[0]));
  edges.col(1) = (positions.col(idI_[2]) - positions.col(idI_[0]));
  P.col(0) = edges.col(0).normalized();
  P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
  Matrix22 F = P.transpose() * edges * rest_;
  Eigen::JacobiSVD<Matrix22> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector2 S = svd.singularValues();
  Vector2 d(0.0f, 0.0f);
  for (int i = 0; i < SHAPEOP_INNER_ITERATIONS; ++i) {
    Scalar v = S(0) * S(1);
    Scalar f = v - clamp(v, rangeMin_, rangeMax_);
    Vector2 g(S(1), S(0));
    d = -((f - g.dot(d)) / g.dot(g)) * g;
    S = svd.singularValues() + d;
  }
  F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
  projections.block<3, 2>(0, idO_) = (weight_ * P * F);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AreaConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 2;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
  }
  idO += n;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE VolumeConstraint::VolumeConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions,
                                                  Scalar rangeMin,
                                                  Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  assert(idI_.size() == 4);
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  rest_ = edges.inverse();
  Scalar V = (edges).determinant() / 6.0f;
  weight_ *= std::sqrt(std::abs(V));
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void VolumeConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix33 edges;
  for (int i = 0; i < 3; ++i) edges.col(i) = positions.col(idI_[i + 1]) - positions.col(idI_[0]);
  Matrix33 F = edges * rest_;
  Eigen::JacobiSVD<Matrix33> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3 S = svd.singularValues();
  Vector3 d(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < SHAPEOP_INNER_ITERATIONS; ++i) {
    Scalar v = S(0) * S(1) * S(2);
    Scalar f = v - clamp(v, rangeMin_, rangeMax_);
    Vector3 g(S(1)*S(2), S(0)*S(2), S(0)*S(1));
    d = -((f - g.dot(d)) / g.dot(g)) * g;
    S = svd.singularValues() + d;
  }
  if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0f) S(2) = -S(2);
  F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
  projections.block<3, 3>(0, idO_) = weight_ * F;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void VolumeConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n = 3;
  for (int i = 0; i < n; ++i) {
    triplets.push_back(Triplet(idO_ + i, idI_[0], -weight_ * (rest_(0, i) + rest_(1, i) + rest_(2, i))));
    triplets.push_back(Triplet(idO_ + i, idI_[1], weight_ * rest_(0, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[2], weight_ * rest_(1, i)));
    triplets.push_back(Triplet(idO_ + i, idI_[3], weight_ * rest_(2, i)));
  }
  idO += n;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE BendingConstraint::BendingConstraint(const std::vector<int> &idI,
                                                    Scalar weight,
                                                    const Matrix3X &positions,
                                                    Scalar rangeMin,
                                                    Scalar rangeMax) :
  Constraint(idI, weight),
  rangeMin_(rangeMin),
  rangeMax_(rangeMax) {
  Matrix3X p(3, idI.size());
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) p.col(i) = positions.col(idI_[i]);
  Scalar l01 = (p.col(0) - p.col(1)).norm();
  Scalar l02 = (p.col(0) - p.col(2)).norm();
  Scalar l12 = (p.col(1) - p.col(2)).norm();
  Scalar r0 = 0.5 * (l01 + l02 + l12);
  Scalar A0 = std::sqrt(r0 * (r0 - l01) * (r0 - l02) * (r0 - l12));
  Scalar l03 = (p.col(0) - p.col(3)).norm();
  Scalar l13 = (p.col(1) - p.col(3)).norm();
  Scalar r1 = 0.5 * (l01 + l03 + l13);
  Scalar A1 = std::sqrt(r1 * (r1 - l01) * (r1 - l03) * (r1 - l13));
  weight_ *= std::sqrt(3.0 / (A0 + A1));
  Scalar cot02 = ((l01 * l01) - (l02 * l02) + (l12 * l12)) / (4.0 * A0);
  Scalar cot12 = ((l01 * l01) + (l02 * l02) - (l12 * l12)) / (4.0 * A0);
  Scalar cot03 = ((l01 * l01) - (l03 * l03) + (l13 * l13)) / (4.0 * A1);
  Scalar cot13 = ((l01 * l01) + (l03 * l03) - (l13 * l13)) / (4.0 * A1);
  w_ =  Vector4::Zero();
  w_(0) = cot02 + cot03;
  w_(1) = cot12 + cot13;
  w_(2) = -(cot02 + cot12);
  w_(3) = -(cot03 + cot13);
  n_ = (p * w_).norm();
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void BendingConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Vector3 e = Vector3::Zero();
  if (n_ > 1e-6) {
    for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
      e += w_(i) * positions.col(idI_[i]);
    Scalar l = e.norm();
    if (l > 1e-6) {
      e /= l;
      l = n_ * clamp(l / n_, rangeMin_, rangeMax_);
      e *= l;
    }
  }
  projections.col(idO_) = weight_ * e;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void BendingConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i)
    triplets.push_back(Triplet(idO_, idI_[i], weight_ * w_(i)));
  idO += 1;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE ClosenessConstraint::ClosenessConstraint(const std::vector<int> &idI,
                                                        Scalar weight,
                                                        const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 1);
  rest_ = positions.col(idI_[0]);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void ClosenessConstraint::project(const Matrix3X & /*positions*/, Matrix3X &projections) const {
  projections.col(idO_) = rest_ * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void ClosenessConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  triplets.push_back(Triplet(idO_, idI_[0], weight_));
  idO += 1;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void ClosenessConstraint::setPosition(const Vector3 &position) {
  rest_ = position;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE Vector3 ClosenessConstraint::getPosition() const {
  return rest_;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE LineConstraint::LineConstraint(const std::vector<int> &idI,
                                              Scalar weight,
                                              const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 2);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void LineConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  Eigen::JacobiSVD<Matrix3X> jSVD;
  jSVD.compute(input, Eigen::ComputeFullU);
  Matrix33 basis = jSVD.matrixU();
  input = basis.transpose() * input;
  input.row(1) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  projections.block(0, idO_, 3, input.cols()) = (basis * input) * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void LineConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = static_cast<int>(idI_.size());
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE PlaneConstraint::PlaneConstraint(const std::vector<int> &idI,
                                                Scalar weight,
                                                const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 3);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void PlaneConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  Eigen::JacobiSVD<Matrix3X> jSVD;
  jSVD.compute(input, Eigen::ComputeFullU);
  Matrix33 basis = jSVD.matrixU();
  input = basis.transpose() * input;
  input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  projections.block(0, idO_, 3, input.cols()) = (basis * input) * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void PlaneConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = static_cast<int>(idI_.size());
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE CircleConstraint::CircleConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 3);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void CircleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  Eigen::JacobiSVD<Matrix3X> jSVD;
  jSVD.compute(input, Eigen::ComputeFullU);
  Matrix33 basis = jSVD.matrixU();
  input = basis.transpose() * input;
  input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  ////////////// 2D Circle fitting
  double Suu = 0.0;
  double Suv = 0.0;
  double Svv = 0.0;
  double Suuu = 0.0;
  double Suvv = 0.0;
  double Svuu = 0.0;
  double Svvv = 0.0;
  for (int j = 0; j < input.cols(); ++j) {
    double uu = input(0, j) * input(0, j);
    double vv = input(1, j) * input(1, j);
    Suu += uu;
    Svv += vv;
    Suv += input(0, j) * input(1, j);
    Suuu += uu * input(0, j);
    Suvv += input(0, j) * vv;
    Svuu += input(1, j) * uu;
    Svvv += vv * input(1, j);
  }
  Matrix22 A;
  A << Suu, Suv,  Suv, Svv;
  if (std::fabs(A.determinant()) > 1e-5) {
    Vector2 b(0.5 * (Suuu + Suvv), 0.5 * (Svvv + Svuu));
    Vector2 center = A.inverse() * b;
    double radius = std::sqrt(center(0) * center(0) + center(1) * center(1) + (Suu + Svv) / static_cast<double>(input.cols()));
    for (int j = 0; j < input.cols(); ++j) {
      Vector2 d = input.block(0, j, 2, 1) - center;
      d.normalize();
      input.block(0, j, 2, 1) = center + d * radius;
    }
  }
  projections.block(0, idO_, 3, input.cols()) = (basis * input) * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void CircleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = static_cast<int>(idI_.size());
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE SphereConstraint::SphereConstraint(const std::vector<int> &idI,
                                                  Scalar weight,
                                                  const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() >= 4);
  input = Matrix3X::Zero(3, idI.size());
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SphereConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  ////////////// 3D Sphere fitting
  double Suu = 0.0;
  double Suv = 0.0;
  double Suw = 0.0;
  double Svv = 0.0;
  double Svw = 0.0;
  double Sww = 0.0;
  double Suuu = 0.0;
  double Suvv = 0.0;
  double Suww = 0.0;
  double Svuu = 0.0;
  double Svvv = 0.0;
  double Svww = 0.0;
  double Swuu = 0.0;
  double Swvv = 0.0;
  double Swww = 0.0;
  for (int j = 0; j < input.cols(); ++j) {
    double uu = input(0, j) * input(0, j);
    double vv = input(1, j) * input(1, j);
    double ww = input(2, j) * input(2, j);
    Suu += uu;
    Svv += vv;
    Sww += ww;
    Suv += input(0, j) * input(1, j);
    Suw += input(0, j) * input(2, j);
    Svw += input(1, j) * input(2, j);
    Suuu += input(0, j) * uu;
    Suvv += input(0, j) * vv;
    Suww += input(0, j) * ww;
    Svuu += input(1, j) * uu;
    Svvv += input(1, j) * vv;
    Svww += input(1, j) * ww;
    Swuu += input(2, j) * uu;
    Swvv += input(2, j) * vv;
    Swww += input(2, j) * ww;
  }
  Matrix33 A;
  A << Suu, Suv, Suw,  Suv, Svv, Svw, Suw, Svw, Sww;
  if (std::fabs(A.determinant()) > 1e-5) {
    Vector3 b(0.5 * (Suuu + Suvv + Suww), 0.5 * (Svuu + Svvv + Svww), 0.5 * (Swuu + Swvv + Swww));
    Vector3 center = A.inverse() * b;
    double radius = std::sqrt(center(0) * center(0) + center(1) * center(1) + center(2) * center(2) + (Suu + Svv + Sww) / static_cast<double>(input.cols()));
    for (int j = 0; j < input.cols(); ++j) {
      Vector3 d = input.col(j) - center;
      d.normalize();
      input.col(j) = center + d * radius;
    }
  }
  projections.block(0, idO_, 3, input.cols()) = input * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SphereConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = static_cast<int>(idI_.size());
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE SimilarityConstraint::SimilarityConstraint(const std::vector<int> &idI,
                                                          Scalar weight,
                                                          const Matrix3X &positions,
                                                          bool scaling /*= true*/,
                                                          bool rotate /*=true*/,
                                                          bool flip /*=true*/) :

  Constraint(idI, weight), scaling_(scaling), rotate_(rotate), flip_(flip) {
  assert(idI.size() >= 2);
  input = Matrix3X::Zero(3, idI.size());
  candidate = Matrix3X::Zero(3, idI.size());
  output = Matrix3X::Zero(3, idI.size());
  Matrix3X shape = Matrix3X::Zero(3, idI.size());
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) shape.col(i) = positions.col(idI_[i]);
  std::vector<Matrix3X> shapes;
  shapes.push_back(shape);
  setShapes(shapes);

  permutation_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  rotateMatrix_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  rotateMatrix_.indices().coeffRef(0) = static_cast<int>(idI.size()) - 1;
  for (int i = 1; i < static_cast<int>(idI.size()); ++i)
    rotateMatrix_.indices().coeffRef(i) = i - 1;
  flipMatrix_ = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(static_cast<int>(idI.size()));
  for (int i = 0; i < static_cast<int>(idI.size()); ++i)
    flipMatrix_.indices().coeffRef(i) = static_cast<int>(idI.size()) - 1 - i;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SimilarityConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;

  Scalar min_error = std::numeric_limits<Scalar>::max();
  permutation_.setIdentity();
  testCandidates(min_error);
  if (rotate_) {
    for (int i = 0; i < input.cols() - 1; ++i) {
      permutation_ = permutation_ * rotateMatrix_; //Rotate
      testCandidates(min_error);
    }
  }
  if (flip_) {
    permutation_.setIdentity();
    permutation_ = permutation_ * flipMatrix_; //Flip
    testCandidates(min_error);
    if (rotate_) {
      for (int i = 0; i < input.cols() - 1; ++i) {
        permutation_ = permutation_ * rotateMatrix_; //Rotate
        testCandidates(min_error);
      }
    }
  }
  projections.block(0, idO_, 3, input.cols()) = output * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SimilarityConstraint::testCandidates(Scalar &min_error) const {
  for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
    candidate = Eigen::umeyama(shapes_[i] * permutation_, input, scaling_).block<3, 3>(0, 0) * shapes_[i] * permutation_;
    Scalar error = (input - candidate).squaredNorm();
    if (error < min_error) {
      min_error = error;
      output = candidate;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SimilarityConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = static_cast<int>(idI_.size());
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = -weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void SimilarityConstraint::setShapes(const std::vector<Matrix3X> &shapes) {
  shapes_ = shapes;
  for (int i = 0; i < static_cast<int>(shapes_.size()); ++i) {
    Vector3 mean_vector = shapes_[i].rowwise().mean();
    shapes_[i].colwise() -= mean_vector;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE RectangleConstraint::RectangleConstraint(const std::vector<int> &idI,
                                                        Scalar weight,
                                                        const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 4);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void RectangleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix34 input;
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  Eigen::JacobiSVD<Matrix3X> jSVD;
  jSVD.compute(input, Eigen::ComputeFullU);
  Matrix33 basis = jSVD.matrixU();
  input = basis.transpose() * input;
  input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  //Rectangular Fit
  MatrixT<8, 6> A;
  A << 1.0, 0.0, 0.0, 0.0, input(0, 0), input(1, 0),
  1.0, 0.0, 0.0, 0.0, input(0, 1), input(1, 1),
  0.0, 1.0, 0.0, 0.0, input(1, 1), -input(0, 1),
  0.0, 1.0, 0.0, 0.0, input(1, 2), -input(0, 2),
  0.0, 0.0, 1.0, 0.0, input(0, 2), input(1, 2),
  0.0, 0.0, 1.0, 0.0, input(0, 3), input(1, 3),
  0.0, 0.0, 0.0, 1.0, input(1, 3), -input(0, 3),
  0.0, 0.0, 0.0, 1.0, input(1, 0), -input(0, 0);
  MatrixT<8, 6> R = A.householderQr().matrixQR().triangularView<Eigen::Upper>();
  Eigen::JacobiSVD<Matrix22> jSVD2D;
  jSVD2D.compute(R.block<2, 2>(4, 4), Eigen::ComputeFullV);
  Matrix22 vec2d = jSVD2D.matrixV();
  Vector2 normal = vec2d.block<2, 1>(0, 1);
  Vector4 center = -R.block<4, 4>(0, 0).inverse() * (R.block<4, 2>(0, 4) * normal);
  //Compute the 4 corner
  Matrix22 B;
  B << normal(0), -normal(1), normal(1), normal(0);
  MatrixT<2, 4> C;
  C << center(0), center(0), center(2), center(2),
  center(3), center(1), center(1), center(3);
  MatrixT<2, 4> corners = -B * C;
  ////////
  input.block<2, 4>(0, 0) = corners;
  projections.block<3, 4>(0, idO_) = (basis * input) * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void RectangleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = 4;
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE ParallelogramConstraint::ParallelogramConstraint(const std::vector<int> &idI,
                                                                Scalar weight,
                                                                const Matrix3X &positions) :
  Constraint(idI, weight) {
  assert(idI.size() == 4);
  MatrixT<8, 6> H;
  H(0, 0) = 1.0; H(0, 1) = 0.0; H(0, 2) = 0.0; H(0, 3) = 0.0; H(0, 4) = 1.0; H(0, 5) = 0.0;
  H(1, 0) = 0.0; H(1, 1) = 1.0; H(1, 2) = 0.0; H(1, 3) = 0.0; H(1, 4) = 0.0; H(1, 5) = 1.0;
  H(2, 0) = 0.0; H(2, 1) = 0.0; H(2, 2) = 1.0; H(2, 3) = 0.0; H(2, 4) = 1.0; H(2, 5) = 0.0;
  H(3, 0) = 0.0; H(3, 1) = 0.0; H(3, 2) = 0.0; H(3, 3) = 1.0; H(3, 4) = 0.0; H(3, 5) = 1.0;
  H(4, 0) = -1.0; H(4, 1) = 0.0; H(4, 2) = 0.0; H(4, 3) = 0.0; H(4, 4) = 1.0; H(4, 5) = 0.0;
  H(5, 0) = 0.0; H(5, 1) = -1.0; H(5, 2) = 0.0; H(5, 3) = 0.0; H(5, 4) = 0.0; H(5, 5) = 1.0;
  H(6, 0) = 0.0; H(6, 1) = 0.0; H(6, 2) = -1.0; H(6, 3) = 0.0; H(6, 4) = 1.0; H(6, 5) = 0.0;
  H(7, 0) = 0.0; H(7, 1) = 0.0; H(7, 2) = 0.0; H(7, 3) = -1.0; H(7, 4) = 0.0; H(7, 5) = 1.0;
  parallelogramMatrix = H * (H.transpose() * H).inverse() * H.transpose();
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void ParallelogramConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  Matrix34 input;
  for (int i = 0; i < static_cast<int>(idI_.size()); ++i) input.col(i) = positions.col(idI_[i]);
  Vector3 mean_vector = input.rowwise().mean();
  input.colwise() -= mean_vector;
  Eigen::JacobiSVD<Matrix3X> jSVD;
  jSVD.compute(input, Eigen::ComputeFullU);
  Matrix33 basis = jSVD.matrixU();
  input = basis.transpose() * input;
  input.row(2) = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>::Constant(input.cols(), 0.0);
  //Check that the coordinate system is in the good direction if not flip it
  bool flip = (input(0, 0) * input(1, 1) - input(0, 1) * input(1, 0)) < 0.0;
  if (flip)
    for (int j = 0; j < 4; ++j)
      input(1, j) = -input(1, j);
  //////
  MatrixT<8, 1> o;
  for (int j = 0; j < 4; ++j) {
    o(j * 2 + 0, 0) = input(0, j);
    o(j * 2 + 1, 0) = input(1, j);
  }
  //mutliply the 3 2d points stacked in o with deformation, corresponds to projection(o)
  MatrixT<8, 1> r = parallelogramMatrix * o;
  //reprojection into 3d
  Matrix34 output;
  for (int j = 0; j < 4; ++j) {
    output(0, j) = r(j * 2 + 0, 0);
    if (flip)
      output(1, j) = -r(j * 2 + 1, 0);
    else
      output(1, j) = r(j * 2 + 1, 0);
    output(2, j) = 0.0;
  }
  projections.block<3, 4>(0, idO_) = (basis * output) * weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void ParallelogramConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  int n_idx = 4;
  double coef1 = (1.0 - 1.0 / n_idx) * weight_;
  double coef2 = - weight_ / n_idx;
  for (int i = 0; i < n_idx; ++i) {
    for (int j = 0; j < n_idx; ++j)
      triplets.push_back(Triplet(idO, idI_[j], (i == j ? coef1 : coef2)));
    idO++;
  }
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE UniformLaplacianConstraint::UniformLaplacianConstraint(const std::vector<int> &idI,
                                                                      Scalar weight,
                                                                      const Matrix3X &positions,
                                                                      bool displacement_lap) :
  Constraint(idI, weight) {
  weighted_rest_.setZero();
  if (displacement_lap) {
    int n_idx = static_cast<int>(idI.size());
    for (int i = 1; i < n_idx; ++i) weighted_rest_ += positions.col(idI[i]);
    weighted_rest_ /= double(n_idx - 1);
    weighted_rest_ -= positions.col(idI[0]);
    weighted_rest_ *= weight_;
  }
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void UniformLaplacianConstraint::project(const Matrix3X & /*positions*/, Matrix3X &projections) const {
  projections.col(idO_) = weighted_rest_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void UniformLaplacianConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  triplets.push_back(Triplet(idO_, idI_[0], -weight_));
  double c = weight_ / (idI_.size() - 1);
  for (int i = 1; i < static_cast<int>(idI_.size()); ++i)
    triplets.push_back(Triplet(idO_, idI_[i], c));
  idO ++;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE AngleConstraint::AngleConstraint(const std::vector<int> &idI,
                                                Scalar weight,
                                                const Matrix3X &positions,
                                                Scalar minAngle,
                                                Scalar maxAngle) :
  Constraint(idI, weight) {
  setMinAngle(minAngle);
  setMaxAngle(maxAngle);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AngleConstraint::project(const Matrix3X &positions, Matrix3X &projections) const {
  assert(minAngleCos_ >= maxAngleCos_);

  Vector3 v1 = positions.col(idI_[1]) - positions.col(idI_[0]),
          v2 = positions.col(idI_[2]) - positions.col(idI_[0]);

  projections.col(idO_) = v1;
  projections.col(idO_ + 1) = v2;

  double epsilon = 1e-14;
  double v1_sqrnorm = v1.squaredNorm(), v2_sqrnorm = v2.squaredNorm();
  double v1_norm = v1.norm(), v2_norm = v2.norm();

  Vector3 unit_v1 = v1, unit_v2 = v2;
  unit_v1.normalize();
  unit_v2.normalize();

  if (unit_v1.allFinite() && unit_v2.allFinite()) {
    // cosine value of the angle gamma between v1 and v2
    double cos_gamma = clamp(unit_v1.dot(unit_v2), -1.0, 1.0);

    // Proceed only when the current angle lies outside the target range, and v1, v2 are not colinear
    if ( (1.0 - std::abs(cos_gamma) > epsilon) && (cos_gamma > minAngleCos_ || cos_gamma < maxAngleCos_) ) {
      double gamma = std::acos(cos_gamma);

      // Angle eta: the sum of displacement angles from v1 and v2
      double eta = cos_gamma > minAngleCos_ ? (minAngle_ - gamma) : (gamma - maxAngle_);
      eta = (std::max)(eta, 0.0);

      // Compute angle theta between v1 and its projection, and angle phi between v2 and its projection
      double theta = 0.5 * std::atan2(v2_sqrnorm * std::sin(2 * eta), v1_sqrnorm + v2_sqrnorm * std::cos(2 * eta));
      theta = (std::max)(0.0, (std::min)(eta, theta));
      double phi = eta - theta;

      // Compute unit vectors that are coplanar with v1, v2, and orthogonal to one of them.
      // They form orthogonal frames with v1 and v2 respectively, within which we compute the projection using the above angles
      Vector3 unit_v3 = unit_v2 - unit_v1 * cos_gamma, unit_v4 = unit_v1 - unit_v2 * cos_gamma;
      unit_v3.normalize();
      unit_v4.normalize();

      // Determine if v1, v2 should move away from each other or towards each other
      if (cos_gamma > minAngleCos_) {
        unit_v3 *= -1.0;
        unit_v4 *= -1.0;
      }

      projections.col(idO_) = (unit_v1 * std::cos(theta) + unit_v3 * std::sin(theta)) * (v1_norm * std::cos(theta));
      projections.col(idO_ + 1) = (unit_v2 * std::cos(phi) + unit_v4 * std::sin(phi)) * (v2_norm * std::cos(phi));
    }
  }

  projections.block<3, 2>(0, idO_) *= weight_;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AngleConstraint::addConstraint(std::vector<Triplet> &triplets, int &idO) const {
  idO_ = idO;
  triplets.push_back(Triplet(idO_, idI_[0], -weight_));
  triplets.push_back(Triplet(idO_, idI_[1], weight_));
  triplets.push_back(Triplet(idO_ + 1, idI_[0], -weight_));
  triplets.push_back(Triplet(idO_ + 1, idI_[2], weight_));
  idO += 2;
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AngleConstraint::setMinAngle(Scalar minAngle) {
  // Ensure the angle limits are between 0 and PI
  // Use parentheses to avoid conflicts with the min/max macros from windows.h
  minAngle_ = (std::max)(minAngle, 0.0);
  minAngleCos_ = clamp(std::cos(minAngle_), -1.0, 1.0);
}
///////////////////////////////////////////////////////////////////////////////
SHAPEOP_INLINE void AngleConstraint::setMaxAngle(Scalar maxAngle) {
  maxAngle_ = (std::min)(maxAngle, M_PI);
  maxAngleCos_ = clamp(std::cos(maxAngle_), -1.0, 1.0);
}
///////////////////////////////////////////////////////////////////////////////
} // namespace ShapeOp
///////////////////////////////////////////////////////////////////////////////
