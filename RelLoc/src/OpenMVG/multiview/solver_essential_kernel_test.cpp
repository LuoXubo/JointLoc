
// Copyright (c) 2010 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.


// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "../multiview/essential.hpp"
#include "../multiview/solver_essential_kernel.hpp"
#include "../multiview/projection.hpp"
#include "../../testing/testing.h"

#include "../multiview/test_data_sets.hpp"

using namespace openMVG;

/// Check that the E matrix fit the Essential Matrix properties
/// Determinant is 0
///
#define EXPECT_ESSENTIAL_MATRIX_PROPERTIES(E, expectedPrecision) { \
  EXPECT_NEAR(0, E.determinant(), expectedPrecision); \
  Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E; \
  Mat3 zero3x3 = Mat3::Zero(); \
  EXPECT_MATRIX_NEAR(zero3x3, O, expectedPrecision);\
}

TEST(EightPointsRelativePose, EightPointsRelativePose_Kernel_IdFocal) {

  //-- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasRing(iNviews, 8,
    nViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  for(int i=0; i <iNviews; ++i)
  {
    vector<Mat3> Es; // Essential,
    vector<Mat3> Rs;  // Rotation matrix.
    vector<Vec3> ts;  // Translation matrix.
    essential::kernel::EightPointRelativePoseSolver::Solve(d._x[i], d._x[(i+1)%iNviews], &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d._x[i].col(0);
      x2Col << d._x[(i+1)%iNviews].col(0);
      CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d._K[i], x1Col,
        d._K[(i+1)%iNviews], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d._R[i], d._t[i], d._R[(i+1)%iNviews], d._t[(i+1)%iNviews], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
    CHECK(bsolution_found);
  }
}

TEST(EightPointsRelativePose, EightPointsRelativePose_Kernel) {

  typedef essential::kernel::EightPointKernel Kernel;

  int focal = 1000;
  int principal_Point = 500;

  //-- Setup a circular camera rig and assert that 8PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasRing(iNviews, Kernel::MINIMUM_SAMPLES,
    nViewDatasetConfigurator(focal,focal,principal_Point,principal_Point,5,0)); // Suppose a camera with Unit matrix as K

  for(int i=0; i <iNviews; ++i)
  {
    vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
    vector<Vec3> ts;      // Translation matrix.

    // Direct value do not work.
    // As we use reference, it cannot convert Mat2X& to Mat&
    Mat x0 = d._x[i];
    Mat x1 = d._x[(i+1)%iNviews];

    Kernel kernel(x0, x1, d._K[i], d._K[(i+1)%iNviews]);
    vector<size_t> samples;
    for (size_t k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d._x[i].col(0);
      x2Col << d._x[(i+1)%iNviews].col(0);
      CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d._K[i], x1Col,
        d._K[(i+1)%iNviews], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d._R[i], d._t[i], d._R[(i+1)%iNviews], d._t[(i+1)%iNviews], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
    CHECK(bsolution_found);
  }
}

TEST(FivePointKernelTest, KernelError) {

  Mat x1(2, 5), x2(2, 5);
  x1 << 0,   0,  0, .8, .8,
        0, -.5, .8,  0, .8;
  x2 << 0,    0,  0, .8, .8,
        .1, -.4, .9,  .1, .9; // Y Translated camera.
  typedef essential::kernel::FivePointKernel Kernel;
  Kernel kernel(x1,x2, Mat3::Identity(), Mat3::Identity());

  bool bOk = true;
  vector<size_t> samples;
  for (size_t i = 0; i < x1.cols(); ++i) {
    samples.push_back(i);
  }
  vector<Mat3> Es;
  kernel.Fit(samples, &Es);

  bOk &= (!Es.empty());
  for (int i = 0; i < Es.size(); ++i) {
    for(int j = 0; j < x1.cols(); ++j)
      EXPECT_NEAR(0.0, kernel.Error(j,Es[i]), 1e-8);
  }
}

TEST(FivePointKernelTest, FivePointsRelativePose_Kernel) {

  typedef essential::kernel::FivePointKernel Kernel;

  int focal = 1000;
  int principal_Point = 500;

  //-- Setup a circular camera rig and assert that 5PT relative pose works.
  const int iNviews = 8;
  NViewDataSet d = NRealisticCamerasRing(iNviews, Kernel::MINIMUM_SAMPLES,
    nViewDatasetConfigurator(focal,focal,principal_Point,principal_Point,5,0)); // Suppose a camera with Unit matrix as K

  size_t found = 0;
  for(int i=1; i <iNviews; ++i)
  {
    vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
    vector<Vec3> ts;      // Translation matrix.

    // Direct value do not work.
    // As we use reference, it cannot convert Mat2X& to Mat&
    Mat x0 = d._x[0];
    Mat x1 = d._x[i];

    Kernel kernel(x0, x1, d._K[0], d._K[1]);
    vector<size_t> samples;
    for (size_t k = 0; k < Kernel::MINIMUM_SAMPLES; ++k) {
      samples.push_back(k);
    }
    kernel.Fit(samples, &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (int s = 0; s < Es.size(); ++s) {
      Vec2 x1Col, x2Col;
      x1Col << d._x[0].col(0);
      x2Col << d._x[i].col(0);
      CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d._K[0], x1Col,
        d._K[i], x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d._R[0], d._t[0], d._R[i], d._t[i], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-4);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
    CHECK(bsolution_found);
    if (bsolution_found)
      found++;
  }
  CHECK_EQUAL(iNviews-1, found);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
