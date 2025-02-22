// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef BUFFER_H
#define BUFFER_H

#include <mars/sensors/sensor_abs_class.h>
#include <mars/time.h>
#include <mars/type_definitions/buffer_entry_type.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <set>

namespace mars
{
///
/// \brief BufferClass that holds mars::BufferEntryType elements and provides access methods
/// \author Christian Brommer <christian.brommer@ieee.org>
/// \attention Erasing elements that are not located at the end or beginning of the buffer will
/// invalidate the deque iterator
///
class Buffer
{
public:
  ///
  /// \brief Buffer default constructor
  /// max buffer size is set to 400 by default
  ///
  Buffer() = default;

  ///
  /// \brief Buffer constructor
  /// \param size max buffer size
  ///
  Buffer(const int& size);

  ///
  /// \brief set_max_buffer_size
  /// \param size max number of entrys after which the oldest entry is deleted
  ///
  void set_max_buffer_size(const int& size);

  ///
  /// \brief get_max_buffer_size
  /// \return current setting for the highest buffer size
  ///
  int get_max_buffer_size() const;

  ///
  /// \brief Removes all entrys from the buffer
  ///
  void ResetBufferData();

  ///
  /// \brief IsEmpty
  /// \return True if the buffer is empty, false otherwise
  ///
  bool IsEmpty() const;

  ///
  /// \brief get_length
  /// \return current number of elements stored in the buffer
  ///
  int get_length() const;

  ///
  /// \brief PrintBufferEntrys prints all buffer entrys in a formated way
  ///
  void PrintBufferEntrys() const;

  ///
  /// \brief get_latest_entry returns the last buffer entry
  /// \param output parameter for the latest entry
  /// \return true if the operation was performed, false otherwise
  ///
  bool get_latest_entry(BufferEntryType* entry) const;

  ///
  /// \brief get_latest_state returns the most recent state entry
  /// \param output parameter for the latest state entry
  /// \return true if the operation was performed, false otherwise
  ///
  bool get_latest_state(BufferEntryType* entry) const;

  ///
  /// \brief get_oldest_state Gets the oldest entry with metadata in the state group
  /// \param entry
  /// \return true if the operation was successfully, false otherwise
  ///
  bool get_oldest_state(BufferEntryType* entry) const;

  ///
  /// \brief get_oldest_core_state Gets the oldest entry with metadata_ core_state
  /// \param entry output variable for the oldest state entry
  /// \return true if the operation was successfully, false otherwise
  ///
  bool get_oldest_core_state(BufferEntryType* entry) const;

  ///
  /// \brief Gets the latest entry with init_state as metadata
  /// \param entry output parameter for the latest init_state entry
  /// \return true if the operation was performed, false otherwise
  ///
  bool get_latest_init_state(BufferEntryType* entry) const;

  ///
  /// \brief get_latest_sensor_handle_state
  /// \param sensor_handle seach parameter for the latest assosiated state entry
  /// \param entry output parameter for the latest sensor handle state entry
  /// \param index, returns the current index of this entry
  /// \return true if the operation was performed, false otherwise
  ///
  bool get_latest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry) const;
  bool get_latest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry,
                                      int* index) const;

  ///
  /// \brief get_oldest_sensor_handle_state
  /// \param sensor_handle seach parameter for the oldest assosiated state entry
  /// \param entry output parameter for the oldest sensor handle state entry
  /// \return true if the operation was successfully, false otherwise
  ///
  bool get_oldest_sensor_handle_state(std::shared_ptr<SensorAbsClass> sensor_handle, BufferEntryType* entry) const;

  ///
  /// \brief get_latest_sensor_handle_measurement
  /// \param sensor_handle seach parameter for the latest associated measurement entry
  /// \param entry output parameter for the latest sensor handle measurement entry
  /// \param index, returns the current index of this entry
  /// \return true if the operation was performed, false otherwise
  ///
  bool get_latest_sensor_handle_measurement(std::shared_ptr<SensorAbsClass> sensor_handle,
                                            BufferEntryType* entry) const;

  ///
  /// \brief get_closest_state
  /// \param timestamp
  /// \param entry
  /// \return true if the operation was performed, false otherwise
  /// \note The newer entry is returned if the time distance between two entrys is equal
  ///
  bool get_closest_state(const Time& timestamp, BufferEntryType* entry) const;
  bool get_closest_state(const Time& timestamp, BufferEntryType* entry, int* index) const;

  ///
  /// \brief get_entry_at_idx
  /// \param index
  /// \param entry
  /// \return
  ///
  bool get_entry_at_idx(const int& index, BufferEntryType* entry) const;

  ///
  /// \brief AddEntrySorted Adds a new entry to the buffer and ensures the buffer is sorted
  /// \param new_entry new buffer entry to be added
  /// \return Index of the added entry. The index is -1 if the entry was removed because the max_buffer_size was
  /// reached.
  ///
  int AddEntrySorted(const BufferEntryType& new_entry);

  ///
  /// \brief FindClosestTimestamp Returns the index of the entry which is the closest to the specified 'timestamp' when
  /// searching from newest to oldest entry
  /// \param timestamp for the surch routine
  /// \return index of the closest entry
  ///
  int FindClosestTimestamp(const Time& timestamp) const;

  ///
  /// \brief Deletes all states after, and including the given index
  /// \param start index after which all states are deleted
  /// \return true if function was performed correct, false otherwise
  ///
  bool DeleteStatesStartingAtIdx(const int& idx);

  ///
  /// \brief Checks if all buffer entrys are correctly sorted by time
  /// \return true if sorted, false otherwise
  ///
  bool IsSorted() const;

  ///
  /// \brief Inserting new element before the element at the specified position
  /// \param entry that is added to the buffer
  ///
  /// The method finds the closest timestamp based on the shortest 'time' distance between the new and existing buffer
  /// elements. It then determines if the entry needs to be added before or after the closest entry.
  ///
  int InsertDataAtTimestamp(const BufferEntryType& new_entry);

  ///
  /// \brief InsertDataAtIndex Adds 'entry' at buffer position 'index'
  /// \param entry buffer entry to be added
  /// \param index position at which the entry is added
  ///
  bool InsertDataAtIndex(const BufferEntryType& new_entry, const int& index);

  ///
  /// \brief CheckForLastHandle Checks if the given sensor handle only exists once in the buffer
  /// \param sensor_handle
  /// \return true if current sensor handle is the last in the buffer, false otherwise
  ///
  bool CheckForLastSensorHandle(const std::shared_ptr<SensorAbsClass>& sensor_handle);

  ///
  /// \brief RemoveOverflowEntrys Removes the oldest entries if max buffer size is reached
  ///
  void RemoveOverflowEntrys();

private:
  ///
  /// \brief deque container that holds the buffer entries
  ///
  std::deque<BufferEntryType> data_;

  ///
  /// \brief defines the max size at wich the oldest entry is removed
  ///
  int max_buffer_size_{ 400 };

  ///
  /// \brief If true, the last entry of a sensor state entry will be keept in the buffer.
  /// \note This only keeps sensor states, not measurements or core states
  ///
  bool keep_last_sensor_handle_{ true };

  bool verbose_{ false };  ///< Increased cmd output

  // new_entry_idx = AppendDataEntry(data_entry)
};
}

#endif  // BUFFER_H
