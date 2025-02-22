
#
# External dependencies
#

find_package(${META_PROJECT_NAME} REQUIRED HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../")

#
# Executable name and options
#

# Target name
set(target mars-test)
set(target_lib mars)
message(STATUS "Test ${target}")


#
# Sources
#

set(sources
    main.cpp
    mars_time.cpp
    mars_buffer_type.cpp
    mars_ekf.cpp
    mars_core_state.cpp
    mars_buffer.cpp
    mars_bind_sensor_data.cpp
    mars_imu_sensor.cpp
    mars_position_sensor.cpp
    mars_gps_conversion.cpp
    #mars_gps_sensor.cpp
    mars_pose_sensor.cpp
    mars_vision_sensor.cpp
    mars_position_sensor.cpp
    mars_type_erasure.cpp
    mars_core_logic.cpp
    mars_nearest_cov.cpp
    mars_utils.cpp
    mars_read_csv.cpp
    #eigen_runtime_test.cpp
)


#
# Create executable
#

# Build executable
add_executable(${target}
    ${sources}
)

# Create namespaced alias
add_executable(${META_PROJECT_NAME}::${target} ALIAS ${target})


#
# Project options
#

set_target_properties(${target}
    PROPERTIES
    ${DEFAULT_PROJECT_OPTIONS}
    FOLDER "${IDE_FOLDER}"
)


#
# Include directories
#

target_include_directories(${target}
    PRIVATE
    ${DEFAULT_INCLUDE_DIRECTORIES}
)


#
# Libraries
#

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LIBRARIES}
    ${META_PROJECT_NAME}::${target_lib}
    gmock-dev
)


#
# Compile definitions
#

target_compile_definitions(${target}
    PRIVATE
    ${DEFAULT_COMPILE_DEFINITIONS}
)


#
# Compile options
#

target_compile_options(${target}
    PRIVATE
    ${DEFAULT_COMPILE_OPTIONS}
)


#
# Linker options
#

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LINKER_OPTIONS}
)
