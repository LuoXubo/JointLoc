#ifndef ROBUSTPOSEESTIMATE_H
#define ROBUSTPOSEESTIMATE_H

#include "common/Camera.h"

namespace PoseEstimate {
using namespace camera;

// absolute pose estimation 2D-3D
bool absPose(const Mat2X& pt2D, const Mat3X& pt3D,
             const CameraIntrinsic& camInfo, Mat4& Tcw,
             std::vector<size_t>& vecInliers, double th, double& maxError);

// relative pose estimation 2D-2D
bool relativePose(const Mat2X& xI, const Mat2X& xJ, const CameraIntrinsic& camI,
                  const CameraIntrinsic& camJ, Mat4& Tij,
                  std::vector<size_t>& vecInliers, double& maxError);

Vec3 point2bearing(float x, float y, const Mat3& K);
void points2bearings(const Mat2X& x, const Mat3& K, Mat3X& norm_x);

}  // namespace PoseEstimate

#endif  // ROBUSTPOSEESTIMATE_H
