// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BUFFERENTRYTYPE_H
#define BUFFERENTRYTYPE_H

#include <mars/sensors/sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_data_type.h>
#include <iostream>
#include <set>

namespace BufferMetadataTypes
{
///
/// \brief Possible metadata states
/// \note Ordered by likelihood of occurence such that the performance of the search function can benefit from it.
///
enum BufferMetadataType
{
  invalid,
  core_state,
  sensor_state,
  init_state,
  measurement,     ///< regular measurement
  measurement_ooo  ///< out of order measurement
};
}

namespace mars
{
using BufferMetadataType = BufferMetadataTypes::BufferMetadataType;

class BufferEntryType
{
public:
  Time timestamp_{ 0.0 };
  BufferDataType data_{};
  std::shared_ptr<SensorAbsClass> sensor_{ nullptr };
  int metadata_{ BufferMetadataType::invalid };

  BufferEntryType() = default;

  BufferEntryType(const Time& timestamp, BufferDataType data, std::shared_ptr<SensorAbsClass> sensor,
                  const int& metadata);

  bool operator<(const BufferEntryType& rhs) const;
  bool operator<=(const BufferEntryType& rhs) const;
  bool operator>(const BufferEntryType& rhs) const;
  bool operator>=(const BufferEntryType& rhs) const;

  friend std::ostream& operator<<(std::ostream& out, const BufferEntryType& entry);

  ///
  /// \brief IsState
  /// \return True if the metadata can be found in the metadata_state_filter_. False otherwise.
  ///
  bool IsState() const;

  ///
  /// \brief IsMeasurement
  /// \return True if the metadata can be found in the metadata_measurement_filter_. False otherwise.
  ///
  bool IsMeasurement() const;

private:
  std::set<int> metadata_state_filter_;
  std::set<int> metadata_measurement_filter_;
};
}
#endif  // BUFFERENTRYTYPE_H
