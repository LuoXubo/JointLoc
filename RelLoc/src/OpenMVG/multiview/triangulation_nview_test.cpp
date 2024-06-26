
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

#include "../multiview/triangulation_nview.hpp"
#include "../multiview/test_data_sets.hpp"
#include "../../testing/testing.h"

using namespace openMVG;

TEST(Triangulate_NView, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j) {
      xs.col(j) = d._x[j].col(i);
    }
    Vec4 X;
    TriangulateNView(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head(2) - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

TEST(Triangulate_NViewAlgebraic, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j) {
      xs.col(j) = d._x[j].col(i);
    }
    Vec4 X;
    TriangulateNViewAlgebraic(xs, Ps, &X);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]*X;
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head<2>() - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

TEST(Triangulate_NViewIterative, FiveViews) {
  int nviews = 5;
  int npoints = 6;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints);

  // Collect P matrices together.
  vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j) {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i) {

    Triangulation triangulationObj;
    for (int j = 0; j < nviews; ++j)
    triangulationObj.add(Ps[j], d._x[j].col(i));

    Vec3 X = triangulationObj.compute();
    // Check reprojection error. Should be nearly zero.
    EXPECT_NEAR(triangulationObj.error(X), 0.0, 1e-9);
    for (int j = 0; j < nviews; ++j) {
      Vec3 x_reprojected = Ps[j]* Vec4(X(0), X(1), X(2), 1.0);
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head<2>() - d._x[j].col(i)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
