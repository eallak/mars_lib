// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#include <gmock/gmock.h>
#include <mars/core_state.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars/type_definitions/core_state_type.h>
#include <mars/type_definitions/core_type.h>
#include <Eigen/Dense>

class mars_core_state_test : public testing::Test
{
public:
};

TEST_F(mars_core_state_test, CTOR_CORE_TYPE)
{
  mars::CoreType core_type();
}

TEST_F(mars_core_state_test, CTOR_CORE_STATE_TYPE)
{
  mars::CoreStateType core;

  ASSERT_TRUE(core.p_wi_.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(core.v_wi_.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(core.q_wi_.isApprox(Eigen::Quaterniond::Identity()));
  ASSERT_TRUE(core.b_w_.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(core.b_a_.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(core.w_m_.isApprox(Eigen::Vector3d::Zero()));
  ASSERT_TRUE(core.a_m_.isApprox(Eigen::Vector3d::Zero()));

  ASSERT_TRUE(core.size_true_ == 16);
  ASSERT_TRUE(core.size_error_ == 15);
}

TEST_F(mars_core_state_test, CTOR_CORE_STATE)
{
  mars::CoreState core_state;
  ASSERT_EQ(core_state.propagation_sensor_.get(), nullptr);

  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr = std::make_shared<mars::ImuSensorClass>("IMU");
  core_state.set_propagation_sensor(imu_sensor_sptr);

  ASSERT_EQ(core_state.propagation_sensor_, imu_sensor_sptr);
  std::cout << core_state.propagation_sensor_.get()->name_ << std::endl;
}

TEST_F(mars_core_state_test, SET_NOISE_PARAM)
{
  mars::CoreState core_state;

  // Test default values
  ASSERT_EQ(core_state.n_a_, Eigen::Vector3d::Zero());
  ASSERT_EQ(core_state.n_ba_, Eigen::Vector3d::Zero());
  ASSERT_EQ(core_state.n_w_, Eigen::Vector3d::Zero());
  ASSERT_EQ(core_state.n_bw_, Eigen::Vector3d::Zero());

  // Test nois value setter
  Eigen::Vector3d n_a(1, 2, 3);
  Eigen::Vector3d n_ba(3, 4, 5);
  Eigen::Vector3d n_w(6, 7, 8);
  Eigen::Vector3d n_bw(9, 10, 11);
  core_state.set_noise_std(n_w, n_bw, n_a, n_ba);

  ASSERT_EQ(core_state.n_a_, n_a);
  ASSERT_EQ(core_state.n_ba_, n_ba);
  ASSERT_EQ(core_state.n_w_, n_w);
  ASSERT_EQ(core_state.n_bw_, n_bw);
}

TEST_F(mars_core_state_test, FD_TEST)
{
  Eigen::Quaterniond q_wi;
  q_wi.setIdentity();
  Eigen::Vector3d a_est(1.5, 2.8, 9.81);
  Eigen::Vector3d w_est(0.12, 0.34, 0.56);
  double dt(0.05);

  mars::CoreStateMatrix result;
  result << 1, 0, 0, 0.0500000000000000, 0, 0, -0.000134228102083333, 0.0122685232291667, -0.00347489379583333,
      1.67798935416667e-06, -0.000204452732291667, 5.80209587916667e-05, -0.00125000000000000, 0, 0, 0, 1, 0, 0,
      0.0500000000000000, 0, -0.0122507522781250, -0.000118229856250000, 0.00194426504375000, 0.000204230960281250,
      1.47779856250000e-06, -3.21163691875000e-05, 0, -0.00125000000000000, 0, 0, 0, 1, 0, 0, 0.0500000000000000,
      0.00351717110416667, -0.00184217545833333, -2.36087083333333e-05, -5.85487943750000e-05, 3.08400879166667e-05,
      2.95045416666667e-07, 0, 0, -0.00125000000000000, 0, 0, 0, 1, 0, 0, -0.00805258150000000, 0.490841858333333,
      -0.138482003666667, 0.000134228102083333, -0.0122685232291667, 0.00347489379583333, -0.0500000000000000, 0, 0, 0,
      0, 0, 0, 1, 0, -0.489772682250000, -0.00709438850000000, 0.0791514535000000, 0.0122507522781250,
      0.000118229856250000, -0.00194426504375000, 0, -0.0500000000000000, 0, 0, 0, 0, 0, 0, 1, 0.141023688333333,
      -0.0730273700000000, -0.00141703000000000, -0.00351717110416667, 0.00184217545833333, 2.36087083333333e-05, 0, 0,
      -0.0500000000000000, 0, 0, 0, 0, 0, 0, 0.999463500000000, 0.0280510000000000, -0.0169160000000000,
      -0.0499910583333333, -0.000700850000000000, 0.000423600000000000, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0279490000000000,
      0.999590000000000, 0.00623800000000000, 0.000699150000000000, -0.0499931666666667, -0.000153966666666667, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0.0170840000000000, -0.00576200000000000, 0.999837500000000, -0.000426400000000000,
      0.000146033333333333, -0.0499972916666667, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  mars::CoreState core_state;
  EXPECT_TRUE(result.isApprox(core_state.GenerateFdSmallAngleApprox(q_wi, a_est, w_est, dt)));
}

TEST_F(mars_core_state_test, Q_TEST)
{
  mars::CoreState core_state;

  double dt_lim(0.05);
  Eigen::Quaterniond q_wi;
  q_wi.setIdentity();
  Eigen::Vector3d a_m(0.5, 0.5, 9.81);
  Eigen::Vector3d n_a(0.6, 0.7, 0.8);
  Eigen::Vector3d b_a(0.1, 0.2, 0.3);
  Eigen::Vector3d n_ba(1, 2, 3);
  Eigen::Vector3d w_m(0.5, 0.6, 0.9);
  Eigen::Vector3d n_w(2.5, 2.1, 2.3);
  Eigen::Vector3d b_w(0.4, 0.5, 0.6);
  Eigen::Vector3d n_bw(0.15, 0.25, 0.35);

  //  a_m = [0.5 0.5 9.81].'
  //  n_a = [0.6 0.7 0.8].'
  //  b_a = [0.1 0.2 0.3].'
  //  n_ba = [1 2 3].'
  //  w_m = [0.5 0.6 0.9].'
  //  n_w = [2.5 2.1 2.3].'
  //  b_w = [0.4 0.5 0.6].'
  //  n_bw = [0.15 0.25 0.35].'

  mars::CoreStateMatrix expected_result;
  expected_result << 3.610413021572712e-05, -2.089966437175472e-05, -2.277079679037254e-07, 1.504695383504477e-03,
      -1.045360055527634e-03, -1.135201227586761e-05, 1.200177696495665e-03, 8.365432042320881e-04,
      1.017680360791267e-03, -5.379596000976564e-08, -1.499059244791667e-07, -2.941127470540366e-07,
      -2.083333333333334e-05, 000000000000000, 000000000000000, -2.089966437175472e-05, 4.119192842254256e-05,
      2.256611169231463e-07, -1.044393345598770e-03, 1.651558921297106e-03, 1.124891291802064e-05,
      -1.191755342647903e-03, -8.306950172226939e-04, -1.004029967246935e-03, 5.337609228515627e-08,
      1.487457181803386e-07, 2.903158903917102e-07, 000000000000000, -8.333333333333336e-05, 000000000000000,
      -2.277079679037254e-07, 2.256611169231463e-07, 2.680975064164928e-05, -1.138250785721977e-05,
      1.128957369315686e-05, 8.071538715599309e-04, -1.288585444835910e-05, -8.980943903893495e-06,
      -1.113177225472409e-05, 5.789228515625004e-10, 1.612897406684029e-09, 3.212442871093752e-09, 000000000000000,
      000000000000000, -1.875000000000001e-04, 1.504695383504477e-03, -1.044393345598770e-03, -1.138250785721977e-05,
      7.423009707115694e-02, -5.572252552013398e-02, -6.055325452950488e-04, 7.187934997136011e-02,
      5.025873577157209e-02, 6.107794321688827e-02, -4.300299966796876e-06, -1.199251953125000e-05,
      -2.353511831835938e-05, -1.250000000000000e-03, 000000000000000, 000000000000000, -1.045360055527634e-03,
      1.651558921297106e-03, 1.128957369315686e-05, -5.572252552013398e-02, 7.992841099487139e-02,
      6.004718096311448e-04, -7.146702755614444e-02, -4.997239241499821e-02, -6.020669189085587e-02,
      4.270021699218750e-06, 1.190907928059896e-05, 2.321923484179688e-05, 000000000000000, -5.000000000000001e-03,
      000000000000000, -1.135201227586761e-05, 1.124891291802064e-05, 8.071538715599309e-04, -6.055325452950488e-04,
      6.004718096311448e-04, 3.238152696900407e-02, -7.688361431861944e-04, -5.375159394457803e-04,
      -6.697339347527393e-04, 4.617386718750003e-08, 1.287364908854167e-07, 2.574423632812501e-07, 000000000000000,
      000000000000000, -1.125000000000000e-02, 1.200177696495665e-03, -1.191755342647903e-03, -1.288585444835910e-05,
      7.187934997136011e-02, -7.146702755614444e-02, -7.688361431861944e-04, 3.124936344929229e-01,
      -6.899309360520053e-04, 1.216788817374684e-04, -2.812441406250001e-05, -3.907877604166668e-07,
      2.542513020833333e-07, 000000000000000, 000000000000000, 000000000000000, 8.365432042320881e-04,
      -8.306950172226939e-04, -8.980943903893495e-06, 5.025873577157209e-02, -4.997239241499821e-02,
      -5.375159394457803e-04, -6.899309360520053e-04, 2.205098670272575e-01, 1.082416327926145e-04,
      1.405664062500001e-07, -7.812337239583335e-05, -2.561653645833333e-07, 000000000000000, 000000000000000,
      000000000000000, 1.017680360791267e-03, -1.004029967246935e-03, -1.113177225472409e-05, 6.107794321688827e-02,
      -6.020669189085587e-02, -6.697339347527393e-04, 1.216788817374684e-04, 1.082416327926145e-04,
      2.645051462701566e-01, -4.705078125000000e-08, 1.297200520833333e-07, -1.531243619791667e-04, 000000000000000,
      000000000000000, 000000000000000, -5.379596000976564e-08, 5.337609228515627e-08, 5.789228515625004e-10,
      -4.300299966796876e-06, 4.270021699218750e-06, 4.617386718750003e-08, -2.812441406250001e-05,
      1.405664062500001e-07, -4.705078125000000e-08, 1.125000000000000e-03, 000000000000000, 000000000000000,
      000000000000000, 000000000000000, 000000000000000, -1.499059244791667e-07, 1.487457181803386e-07,
      1.612897406684029e-09, -1.199251953125000e-05, 1.190907928059896e-05, 1.287364908854167e-07,
      -3.907877604166668e-07, -7.812337239583335e-05, 1.297200520833333e-07, 000000000000000, 3.125000000000000e-03,
      000000000000000, 000000000000000, 000000000000000, 000000000000000, -2.941127470540366e-07, 2.903158903917102e-07,
      3.212442871093752e-09, -2.353511831835938e-05, 2.321923484179688e-05, 2.574423632812501e-07,
      2.542513020833333e-07, -2.561653645833333e-07, -1.531243619791667e-04, 000000000000000, 000000000000000,
      6.124999999999999e-03, 000000000000000, 000000000000000, 000000000000000, -2.083333333333334e-05, 000000000000000,
      000000000000000, -1.250000000000000e-03, 000000000000000, 000000000000000, 000000000000000, 000000000000000,
      000000000000000, 000000000000000, 000000000000000, 000000000000000, 5.000000000000000e-02, 000000000000000,
      000000000000000, 000000000000000, -8.333333333333336e-05, 000000000000000, 000000000000000,
      -5.000000000000001e-03, 000000000000000, 000000000000000, 000000000000000, 000000000000000, 000000000000000,
      000000000000000, 000000000000000, 000000000000000, 2.000000000000000e-01, 000000000000000, 000000000000000,
      000000000000000, -1.875000000000001e-04, 000000000000000, 000000000000000, -1.125000000000000e-02,
      000000000000000, 000000000000000, 000000000000000, 000000000000000, 000000000000000, 000000000000000,
      000000000000000, 000000000000000, 4.500000000000000e-01;

  mars::CoreStateMatrix test_return =
      core_state.CalcQSmallAngleApprox(dt_lim, q_wi, a_m, n_a, b_a, n_ba, w_m, n_w, b_w, n_bw);

  EXPECT_TRUE(test_return.isApprox(expected_result));
}
