#ifndef MULTIVIEWCOMPUTE_H
#define MULTIVIEWCOMPUTE_H

#include "common/Types.h"

class MultiViewCompute {
 public:
  MultiViewCompute();

  static Mat34 getPFrmKRt(const Mat3 &K, const Mat3 &R, const Vec3 &t);

  static float cmpDepth(const Mat3 &R, const Vec3 &t, const Vec3 &pt3D);

  static float cmpCosParallax(const Vec3 &C1, const Vec3 &C2, const Vec3 &pt3D);

  static float cmpPrjError(const Mat34 &P, const Vec3 &X, const Vec2 &obs);

  static Vec3 triangulate(const Mat34 &P1, const Vec2 &x1, const Mat34 &P2,
                          const Vec2 &x2);
};

#endif  // MULTIVIEWCOMPUTE_H
