#ifndef TOOLS_PRECISION_EVALUATION_TO_GT_HPP
#define TOOLS_PRECISION_EVALUATION_TO_GT_HPP
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>

#include "modules/OpenMVG/geometry/rigid_transformation3D_srt.hpp"

namespace tools {

double getRotationMagnitude(const Mat3& R2) {
  const Mat3 R1 = Mat3::Identity();
  double cos_theta = (R1.array() * R2.array()).sum() / 3.0;
  cos_theta = openMVG::clamp(cos_theta, -1.0, 1.0);
  return std::acos(cos_theta);
}

bool computeSimilarity(const std::vector<Vec3>& vecCamPosGT,
                       const std::vector<Vec3>& vecCamPosComputed,
                       double& scale, Mat3& R, Vec3& t,
                       size_t nFixedCount = 0) {
  if (vecCamPosGT.size() != vecCamPosComputed.size()) {
    std::cerr << "Cannot perform registration, vector sizes are different"
              << std::endl;
    return false;
  }

  // Move input point in appropriate container
  size_t nCount = nFixedCount == 0 ? vecCamPosGT.size() : nFixedCount;
  nCount = std::min(nCount, vecCamPosGT.size());
  Mat x1(3, vecCamPosGT.size());
  Mat x2(3, vecCamPosGT.size());
  for (size_t i = 0; i < nCount; ++i) {
    x1.col(i) = vecCamPosComputed[i];
    x2.col(i) = vecCamPosGT[i];
  }
  // Compute rigid transformation p'i = S R pi + t

  openMVG::geometry::FindRTS(x1, x2, &scale, &t, &R);

  std::cout << "\nNon linear refinement" << std::endl;

  openMVG::geometry::Refine_RTS(x1, x2, &scale, &t, &R);

  //    std::vector<double> vecResiduals(vecCamPosGT.size());
  //    for (size_t i = 0; i  < vecCamPosGT.size(); ++i)
  //    {
  //        const Vec3 newPos = scale * R * ( vecCamPosComputed[i]) + t;
  //        const double dResidual = (newPos - vecCamPosGT[i]).norm();
  //        vecResiduals[i] = dResidual;
  //        std::cout << "res: " << dResidual << std::endl;
  //    }

  return true;
}

}  // namespace tools

#endif  // TOOLS_PRECISION_EVALUATION_TO_GT_HPP
