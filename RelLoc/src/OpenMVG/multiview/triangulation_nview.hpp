
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

#ifndef OPENMVG_MULTIVIEW_RESECTION_H
#define OPENMVG_MULTIVIEW_RESECTION_H

#include "../numeric/numeric.h"

namespace openMVG {

/// Compute a 3D position of a point from several images of it. In particular,
///  compute the projective point X in R^4 such that x = PX.
/// Algorithm is the standard DLT; for derivation see appendix of Keir's thesis.
void TriangulateNView(
        const Mat2X &x, // x's are 2D coordinates (x,y,1) in each image
        const std::vector< Mat34 > &Ps, // Ps are projective cameras
        Vec4 *X);

void TriangulateNView(
        const Mat2X &x, // x's are 2D coordinates (x,y,1) in each image
        const std::vector< Mat34 > &Ps, // Ps are projective cameras
        Vec3 *X);

// This method uses the algebraic distance approximation.
// Note that this method works better when the 2D points are normalized
// with an isotopic normalization.
void TriangulateNViewAlgebraic(
        const Mat2X &x, // x's are 2D coordinates (x,y,1) in each image
        const std::vector< Mat34 > &Ps, // Ps are projective cameras.
        Vec4 *X);

//Iterated linear method
class Triangulation
{
public:
    Triangulation();

    size_t size() const {	return m_views.size();}

    void clear()  { m_views.clear();}

    void add(const Mat34& camera, const Vec2 & p) {
        m_views.push_back(camera);
        m_vecPts.push_back(p);
    }

    // Return squared L2 sum of error
    double error(const Vec3 &X) const;

    Vec3 compute(int iter = 3);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Accessors

    // These values are defined after a successful call to compute
    double minDepth() const { return m_zmin; }
    double maxDepth() const { return m_zmax; }
    double error()    const { return m_err; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Data members

private:
    Vec3 trigulate();

protected:
    mutable double m_zmin; // min depth, mutable since modified in compute(...) const;
    mutable double m_zmax; // max depth, mutable since modified in compute(...) const;
    mutable double m_err;  // re-projection error, mutable since modified in compute(...) const;
    std::vector< Vec2 > m_vecPts; // Proj matrix and associated image point
    std::vector< Mat34 > m_views; // Proj matrix and associated image point
};

}  // namespace openMVG

#endif  // OPENMVG_MULTIVIEW_RESECTION_H
