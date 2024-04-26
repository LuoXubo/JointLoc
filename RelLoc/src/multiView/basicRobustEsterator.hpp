#ifndef BASIC_ROBUST_ESTERATOR_H
#define BASIC_ROBUST_ESTERATOR_H

#include "OpenMVG/multiview/projection.hpp"
#include "OpenMVG/multiview/solver_essential_kernel.hpp"
#include "OpenMVG/multiview/solver_resection_kernel.hpp"
#include "OpenMVG/multiview/solver_resection_p3p.hpp"
#include "OpenMVG/multiview/triangulation.hpp"
#include "OpenMVG/numeric/numeric.h"
#include "OpenMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "OpenMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"

// #include "ACRansacKernelExten.h"

using namespace openMVG;
using namespace openMVG::robust;

static const size_t ACRANSAC_ITER = 1024;

namespace BasicEstimate {

/// Estimate the essential matrix from point matches and K matrices.
bool estimate_E(const Mat& x1, const Mat& x2, const Mat3& K1, const Mat3& K2,
                const std::pair<size_t, size_t>& size1,
                const std::pair<size_t, size_t>& size2, Mat3& E,
                std::vector<size_t>& vecInlier, double& errorMax) {
  E = Mat3::Identity(3, 3);

  double precision = std::numeric_limits<double>::infinity();

  // Use the 5 point solver to estimate E
  typedef essential::kernel::FivePointKernel SolverType;
  // Define the AContrario adaptor
  typedef ACKernelAdaptorEssential<SolverType,
                                   fundamental::kernel::EpipolarDistanceError,
                                   UnnormalizerT, Mat3>
      KernelType;

  KernelType kernel(x1, size1.first, size1.second, x2, size2.first,
                    size2.second, K1, K2);

  // Robustly estimation of the Essential matrix and it's precision
  std::pair<double, double> ACRansacOut =
      ACRANSAC(kernel, vecInlier, ACRANSAC_ITER, &E, precision, false);
  errorMax = ACRansacOut.first;

  return vecInlier.size() > 2.5 * SolverType::MINIMUM_SAMPLES;
}

/// Estimate the best possible Rotation/Translation from E
/// Four are possible, keep the one with most of the point in front.
bool Rt_frmE(const Mat& x1, const Mat& x2, const Mat3& K1, const Mat3& K2,
             const Mat3& E, const std::vector<size_t>& vecInliers, Mat3& R,
             Vec3& t) {
  std::vector<size_t> f(4, 0);

  std::vector<Mat3> Es;  // Essential,
  std::vector<Mat3> Rs;  // Rotation matrix.
  std::vector<Vec3> ts;  // Translation matrix.

  Es.push_back(E);
  // Recover best rotation and translation from E.
  MotionFromEssential(E, &Rs, &ts);

  //-> Test the 4 solutions will all the point
  assert(Rs.size() == 4);
  assert(ts.size() == 4);

  Mat34 P1, P2;
  Mat3 R1 = Mat3::Identity();
  Vec3 t1 = Vec3::Zero();
  P_From_KRt(K1, R1, t1, &P1);

  bool bOk = false;

  for (int i = 0; i < 4; ++i) {
    const Mat3& R2 = Rs[i];
    const Vec3& t2 = ts[i];
    P_From_KRt(K2, R2, t2, &P2);
    Vec3 X;

    for (size_t k = 0; k < vecInliers.size(); ++k) {
      const Vec2 &x1_ = x1.col(vecInliers[k]), &x2_ = x2.col(vecInliers[k]);
      TriangulateDLT(P1, x1_, P2, x2_, X);
      // Test if point is front to the two cameras.
      if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0) {
        ++f[i];
      }
    }
  }
  // Check the solution :
  //    std::cout << std::endl << "Check [R t] -- points in front of both
  //    cameras:"
  //              << f[0] << " " << f[1] << " " << f[2] << " " << f[3] <<
  //              std::endl;
  std::vector<size_t>::iterator iter = max_element(f.begin(), f.end());
  if (*iter != 0) {
    size_t index = std::distance(f.begin(), iter);
    R = Rs[index];
    t = ts[index];
    bOk = true;
  } else {
    std::cerr << std::endl
              << "/!\\There is no right solution,"
              << " probably intermediate results are not correct or no points"
              << " in front of both cameras" << std::endl;
    bOk = false;
  }
  return bOk;
}

// Compute the residual of the projection distance(pt2D, Project(P,pt3D))
// Return the squared error
struct ResectionSquaredResidualError {
  static double Error(const Mat34& P, const Vec2& pt2D, const Vec3& pt3D) {
    Vec2 x = Project(P, pt3D);
    return (x - pt2D).squaredNorm();
  }
};

/// Compute the robust resection of the 3D<->2D correspondences.
bool ACRansacResection(const Mat& pt2D, const Mat& pt3D,
                       const std::pair<size_t, size_t>& imageSize,
                       std::vector<size_t>& vecInliers, const Mat3* K = NULL,
                       Mat34* P = NULL, double* maxError = NULL,
                       double th = 2.0) {
  vecInliers.clear();
  double dPrecision = th * th;
  size_t MINIMUM_SAMPLES = 0;
  // Classic resection
  if (K == NULL) {
    typedef openMVG::resection::kernel::SixPointResectionSolver SolverType;
    MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;

    typedef ACKernelAdaptorResection<SolverType, ResectionSquaredResidualError,
                                     UnnormalizerResection, Mat34>
        KernelType;

    KernelType kernel(pt2D, imageSize.first, imageSize.second, pt3D);
    // Robustly estimation of the Projection matrix and it's precision
    std::pair<double, double> ACRansacOut =
        ACRANSAC(kernel, vecInliers, ACRANSAC_ITER, P, dPrecision, true);
    *maxError = ACRansacOut.first;

  } else {
    // If K is available use the Epnp solver
    // typedef openMVG::euclidean_resection::kernel::EpnpSolver SolverType;
    typedef openMVG::euclidean_resection::P3PSolver SolverType;
    MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;

    typedef ACKernelAdaptorResection_K<
        SolverType, ResectionSquaredResidualError, UnnormalizerResection, Mat34>
        KernelType;

    KernelType kernel(pt2D, pt3D, *K);
    // Robustly estimation of the Projection matrix and it's precision
    std::pair<double, double> ACRansacOut =
        ACRANSAC(kernel, vecInliers, ACRANSAC_ITER, P, dPrecision, false);
    *maxError = ACRansacOut.first;
  }

  // Test if the found model is valid
  if (vecInliers.size() > 2.5 * MINIMUM_SAMPLES)
    return true;
  else {
    P = NULL;
    return false;
  }
}

/*

bool ACRansacTranslation(const Mat& pt2D, const Mat& pt3D,
                         const Mat3& R, const Mat3& K,
                         Mat34& P,
                         std::vector<size_t>& vecInliers,
                         double& maxError)
{
    vecInliers.clear();
    double dPrecision = std::numeric_limits<double>::infinity();

    typedef robust::ACKernelAdaptorResectionP2P<ResectionSquaredResidualError,
            UnnormalizerResection, Mat34>  KernelType;

    KernelType kernel(pt2D, pt3D, R, K);

 //   std::cout << "pt2D: \n" << pt2D << std::endl;
 //   std::cout << "pt3D: \n" << pt3D << std::endl;

    //ACRACSAC
    std::pair<double,double> results = robust::ACRANSAC(kernel, vecInliers,
                                                ACRANSAC_ITER, &P, dPrecision,
false); maxError = results.first;

    // Test if the found model is valid
    if (vecInliers.size() > 3 * robust::P2PSolver::MINIMUM_SAMPLES)
        return true;
    else{
        P = Mat34::Zero(3,4);
        return false;
    }
}
*/

}  // namespace BasicEstimate

#endif
