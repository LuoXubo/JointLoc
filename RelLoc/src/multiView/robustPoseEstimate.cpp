#include "robustPoseEstimate.h"

#include "basicRobustEsterator.hpp"

Vec3 PoseEstimate::point2bearing(float x, float y, const Mat3& K) {
  Mat3 T = K.inverse();
  Vec3 in, out;
  in << x, y, 1;
  out = T * in;
  out.normalize();
  return out;
}

void PoseEstimate::points2bearings(const Mat2X& x, const Mat3& K,
                                   Mat3X& norm_x) {
  const Mat::Index n = x.cols();
  Mat3 T = K.inverse();
  norm_x.resize(3, n);
  for (Mat::Index i = 0; i < n; ++i) {
    Vec3 in, out;
    in << x(0, i), x(1, i), 1;
    out = T * in;
    out.normalize();
    norm_x.col(i) = out;
  }
}

bool PoseEstimate::absPose(const Mat2X& pt2D, const Mat3X& pt3D,
                           const CameraIntrinsic& camInfo, Mat4& Tcw,
                           std::vector<size_t>& vecInliers, double th,
                           double& maxError) {
  Tcw = Mat4::Identity();
  Mat34 P;
  bool ret =
      BasicEstimate::ACRansacResection(pt2D, pt3D, camInfo.imSize(), vecInliers,
                                       &camInfo.getK(), &P, &maxError, th);
  if (ret) {
    Mat3 K, R;
    Vec3 t;
    KRt_From_P(P, &K, &R, &t);

    Tcw.block<3, 3>(0, 0) = R;
    Tcw.block<3, 1>(0, 3) = t;
  }
  return ret;
}

bool PoseEstimate::relativePose(const Mat2X& xI, const Mat2X& xJ,
                                const CameraIntrinsic& camI,
                                const CameraIntrinsic& camJ, Mat4& Tij,
                                std::vector<size_t>& vecInliers,
                                double& maxError) {
  Mat3 E = Mat3::Identity();

  // std::cout << "Image size: " << camI.imSize().first << ", " <<
  // camI.imSize().second << std::endl; std::cout << "Image size: " <<
  // camJ.imSize().first << ", " << camJ.imSize().second << std::endl;

  if (!BasicEstimate::estimate_E(xI, xJ, camI.getK(), camJ.getK(),
                                 camI.imSize(), camJ.imSize(), E, vecInliers,
                                 maxError))
    return false;

  // Estimate the best possible Rotation/Translation from E
  Mat3 R;
  Vec3 t;
  if (!BasicEstimate::Rt_frmE(xI, xJ, camI.getK(), camJ.getK(), E, vecInliers,
                              R, t))
    return false;

  Tij = Mat4::Identity();
  Tij.block<3, 3>(0, 0) = R;
  Tij.block<3, 1>(0, 3) = t;

  return true;
}
