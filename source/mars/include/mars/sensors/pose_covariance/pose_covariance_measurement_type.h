// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSECOVARIANCEMEASUREMENTTYPE_H
#define POSECOVARIANCEMEASUREMENTTYPE_H

#include <Eigen/Dense>
#include <utility>

namespace mars
{
class PoseCovarianceMeasurementType
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_;               ///< Position [x y z]
  Eigen::Quaternion<double> orientation_;  ///< Quaternion [w x y z]

  Eigen::Matrix<double, 6, 6> covariance_;

  PoseCovarianceMeasurementType() = default;

  PoseCovarianceMeasurementType(Eigen::Vector3d position, const Eigen::Quaternion<double>& orientation, const Eigen::Matrix<double, 6, 6>& covariance)
    : position_(std::move(position)), orientation_(orientation), covariance_(covariance)
  {
  }
};
}
#endif  // POSEMEASUREMENTTYPE_H
