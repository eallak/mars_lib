// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef POSITIONSENSORSTATETYPE_H
#define POSITIONSENSORSTATETYPE_H

#include <mars/type_definitions/base_states.h>
#include <Eigen/Dense>

namespace mars
{
class PositionSensorStateType : public BaseStates
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_ip_;

  PositionSensorStateType() : BaseStates(3)  // cov size
  {
    p_ip_.setZero();
  }
};
}
#endif  // POSITIONSENSORSTATETYPE_H
