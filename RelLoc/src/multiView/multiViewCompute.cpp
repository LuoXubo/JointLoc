#include "multiViewCompute.h"

#include "OpenMVG/multiview/projection.hpp"
#include "OpenMVG/multiview/triangulation.hpp"
#include "OpenMVG/numeric/numeric.h"

MultiViewCompute::MultiViewCompute() {}

Mat34 MultiViewCompute::getPFrmKRt(const Mat3 &K, const Mat3 &R,
                                   const Vec3 &t) {
  return K * openMVG::HStack(R, t);
}

float MultiViewCompute::cmpDepth(const Mat3 &R, const Vec3 &t,
                                 const Vec3 &pt3D) {
  return openMVG::Depth(R, t, pt3D);
}

float MultiViewCompute::cmpCosParallax(const Vec3 &C1, const Vec3 &C2,
                                       const Vec3 &pt3D) {
  Vec3 normal1 = (pt3D - C1).normalized();
  Vec3 normal2 = (pt3D - C2).normalized();

  return normal1.dot(normal2);
}

float MultiViewCompute::cmpPrjError(const Mat34 &P, const Vec3 &X,
                                    const Vec2 &obs) {
  return (obs - openMVG::Project(P, X)).norm();
}

Vec3 MultiViewCompute::triangulate(const Mat34 &P1, const Vec2 &x1,
                                   const Mat34 &P2, const Vec2 &x2) {
  return openMVG::Triangulate(P1, x1, P2, x2);
}
