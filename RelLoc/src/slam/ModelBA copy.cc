
#include "ModelBA.h"

#include <ceres/ceres.h>
#include <ceres/conditioned_cost_function.h>
#include <ceres/iteration_callback.h>
#include <opencv2/core/eigen.hpp>

#include <memory>
#include <thread>
#include <unistd.h>
#include <ctime>
#include <unsupported/Eigen/MatrixFunctions>

#include "BAErrors.h"
#include "Converter.h"
#include "Parameters.h"
#include "Sim3Solver.h"
#include "multiView/multiViewCompute.h"
#include "multiView/robustPoseEstimate.h"
#include "ViewPoint.h"
extern NormalPrameter nprameter;

using namespace ORB_SLAM2;
using namespace BA;
using namespace BASE;

// define a callback
static bool *pForceBAStopFlag = NULL;
class DummyIterationCallback : public ceres::IterationCallback
{
public:
  virtual ~DummyIterationCallback() {}
  virtual ceres::CallbackReturnType
  operator()(const ceres::IterationSummary &summary)
  {
    if (pForceBAStopFlag && *pForceBAStopFlag)
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    else
      return ceres::SOLVER_CONTINUE;
  }
};

void setCeresOption(ceres::Solver::Options &options);
bool setLBAParams(const std::list<KeyFrame *> &lpKFs,
                  const std::list<KeyFrame *> &lFixedKFs,
                  const std::list<MapPoint *> &lpMP, FrameID baseFrameId,
                  BAParameters &baParams);
bool setCam2IMU(const cv::Mat &Tci, const cv::Mat &Rci, BAParameters &baParams);
bool buildLBAProblem(const std::list<MapPoint *> &lLocalMapPoints,
                     const std::vector<ViewID> &vKeyViewIds,
                     BAParameters &baParams, ceres::Problem &problem, float th);
bool buildXYProblem(std::list<KeyFrame *> &lLocalKeyFrames,
                    BAParameters &baParams, ceres::Problem &problem);

bool Optimizer::GlobalBundleAdjustemnt(Map *pMap, FrameID baseFrameId,
                                       const unsigned long nLoopKF,
                                       const bool bRobust)
{
  vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
  vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
  if (vpKFs.size() < 2 || vpMP.size() < 20)
    return false;

  vector<bool> vbIncludedMP(vpMP.size(), false);

  std::list<KeyFrame *> lFrames;
  long unsigned int maxKFid = 0;
  size_t idx = 0;
  BAParameters baParams;
  for (size_t i = 0; i < vpKFs.size(); ++i) // key frame
  {
    KeyFrame *pKF = vpKFs[i];
    if (pKF->isBad())
      continue;

    lFrames.push_back(pKF);
    Vec3 C, A;
    Converter::toSixDoFPose(pKF->GetPose().Tcw(), C, A);
    baParams.param_push_back(C[0]), baParams.param_push_back(C[1]),
        baParams.param_push_back(C[2]);
    baParams.param_push_back(A[0]), baParams.param_push_back(A[1]),
        baParams.param_push_back(A[2]);
    baParams.viwIdx_push_back(pKF->mnId, idx++);
    baParams.viwFixedFlag_push_back(pKF->mnId, pKF->mnId == baseFrameId);
    if (pKF->mnId > maxKFid)
      maxKFid = pKF->mnId;
  }

  if (baParams.num_views() < 2)
    return false;

  for (size_t i = 0, idx = 0; i < vpMP.size(); ++i) // 3D map points
  {
    MapPoint *pMP = vpMP[i];
    if (pMP->isBad())
      continue;
    Vec3 pt3D = Converter::toVector3d(pMP->GetWorldPos());
    baParams.param_push_back(pt3D[0]), baParams.param_push_back(pt3D[1]),
        baParams.param_push_back(pt3D[2]);
    baParams.ptsIdx_push_back(pMP->mnId, idx++);
  }
  if (baParams.num_points() < 20)
    return false;

  // build BA problem
  ceres::HuberLoss *loss =
      bRobust ? new ceres::HuberLoss(Param::TH_CHI_2D) : NULL;
  ceres::Problem problem;
  for (size_t i = 0; i < vpMP.size(); i++)
  {
    MapPoint *pMP = vpMP[i];
    if (pMP->isBad())
      continue;

    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
    int nEdges = 0;
    for (auto mit = observations.begin(); mit != observations.end(); mit++)
    {
      KeyFrame *pKF = mit->first;
      if (pKF->isBad() || pKF->mnId > maxKFid)
        continue;

      if (!baParams.view_exist(pKF->mnId))
      {
        std::cout << "No exist key frame: " << pKF->mnId << std::endl;
        continue;
      }
      nEdges++;

      const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
      if (pKF->mvuRight[mit->second] < 0)
      {
        double cx = pKF->cx, cy = pKF->cy;
        double f = pKF->fx;
        double invSigma = sqrt(pKF->mvInvLevelSigma2[kpUn.octave]);

        ceres::CostFunction *costFun = ReprojectionError::create(
            kpUn.pt.x - cx, kpUn.pt.y - cy, f, invSigma);
        problem.AddResidualBlock(costFun, loss,
                                 baParams.mutable_view(pKF->mnId),
                                 baParams.mutable_view(pKF->mnId) + 1,
                                 baParams.mutable_view(pKF->mnId) + 2,
                                 baParams.mutable_view(pKF->mnId) + 3,
                                 baParams.mutable_view(pKF->mnId) + 4,
                                 baParams.mutable_view(pKF->mnId) + 5,
                                 baParams.mutable_point(pMP->mnId));

        if (baParams.view_fixed(pKF->mnId)) // set fixed frame
        {
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId));
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId) +
                                            1);
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId) +
                                            2);
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId) +
                                            3);
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId) +
                                            4);
          problem.SetParameterBlockConstant(baParams.mutable_view(pKF->mnId) +
                                            5);
        }
      }
    }
    if (nEdges > 1)
      vbIncludedMP[i] = true;
  }

  // for GPS and IMU data
  if (buildXYProblem(lFrames, baParams, problem))
  {
    std::cout << "\nRun GBA with GPS/IMU measurements!" << std::endl;
  }

  ceres::Solver::Options options;
  // for small BA problem, DOGLEG is a more efficient option
  if (baParams.num_views() < 10)
  {
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
  }
  options.max_num_iterations = Param::MAX_ITERATION_NUMBER;
  size_t nthreads = std::thread::hardware_concurrency();
  options.num_threads = nthreads > Param::MAX_THREAD_FOR_SOLVER
                            ? Param::MAX_THREAD_FOR_SOLVER
                            : nthreads;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // BA report
  std::cout << "-------------------------------\n";
  std::cout << "--nViews: " << baParams.num_views() << std::endl
            << "--nPoints3D: " << baParams.num_points() << std::endl;
  std::cout << "--GBA: " << summary.FullReport() << std::endl;
  std::cout << "--GBA time: " << summary.total_time_in_seconds << " s"
            << std::endl;
  std::cout << "-------------------------------\n";

  // Recover data...
  for (size_t i = 0; i < vpKFs.size(); i++)
  {
    KeyFrame *pKF = vpKFs[i];
    if (pKF->isBad())
      continue;
    Mat4 tmpT = Converter::toMatrixPose(baParams.mutable_view(pKF->mnId));

    if (nLoopKF == 0)
      pKF->SetPose(camera::Pose(Converter::toCvMat(tmpT)));
    else
    {
      pKF->mTcwGBA.create(4, 4, CV_32F);
      Converter::toCvMat(tmpT).copyTo(pKF->mTcwGBA);
      pKF->mnBAGlobalForKF = nLoopKF;
    }
  }
  for (size_t i = 0; i < vpMP.size(); i++)
  {
    if (!vbIncludedMP[i])
      continue;
    MapPoint *pMP = vpMP[i];

    const double *pt = baParams.mutable_point(pMP->mnId);
    Vec3 vPoint(pt[0], pt[1], pt[2]);
    if (nLoopKF == 0)
    {
      pMP->SetWorldPos(Converter::toCvMat(vPoint));
      pMP->UpdateNormalAndDepth();
    }
    else
    {
      pMP->mPosGBA.create(3, 1, CV_32F);
      Converter::toCvMat(vPoint).copyTo(pMP->mPosGBA);
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
  return true;
}

int Optimizer::PoseOptimization(Frame *pFrame, bool bRobust)
{
  unique_lock<mutex> lock(MapPoint::mGlobalMutex);
  // run PnP
  //-----------------------------------------------------------------------------------------------------
  camera::CameraIntrinsic camCalib(Converter::toMatrix3d(pFrame->mK),
                                   pFrame->mImCols, pFrame->mImRows);
  std::vector<Vec2> vecFeats;
  std::vector<Vec3> vecMapPts;
  std::vector<int> vecIdx;
  for (int i = 0; i < pFrame->mNumOfKPts; i++)
  {
    MapPoint *pMP = pFrame->mvpMapPoints[i];
    if (pMP == NULL || pFrame->mvbOutlier[i])
      continue;
    if (pFrame->mvuRight[i] < 0)
    {
      const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
      Vec2 obs(kpUn.pt.x, kpUn.pt.y);
      vecFeats.push_back(obs);
      Vec3 pt3D = Converter::toVector3d(pMP->GetWorldPos());
      vecMapPts.push_back(pt3D);
      vecIdx.push_back(i);
    }
  }

  Mat2X matFeats = Mat2X::Zero(2, vecFeats.size());
  Mat3X matMps = Mat3X::Zero(3, vecFeats.size());
  for (size_t i = 0; i < vecFeats.size(); ++i)
  {
    matFeats.col(i) = vecFeats[i];
    matMps.col(i) = vecMapPts[i];
  }
  std::vector<size_t> vecInliers;
  double errorMax = std::numeric_limits<double>::max();
  Mat4 Tcw;
  PoseEstimate::absPose(matFeats, matMps, camCalib, Tcw, vecInliers,
                        Param::TH_CHI_2D, errorMax);

  if (vecInliers.size() < 3)
  {
    std::cout << "Insufficient observations for pose estimation!" << std::endl;
    return 0;
  }
  // std::cout << "ErrorMax = " << errorMax << ", ";
  // std::cout << "vecInliers = " << vecInliers.size() << "/" << vecIdx.size()
  // << std::endl;
  pFrame->SetPose(Converter::toCvMat(Tcw));
  //-----------------------------------------------------------------------------------------------------

  // run optimization
  //-----------------------------------------------------------------------------------------------------
  BAParameters baParams;
  {
    Vec3 C, A;
    Converter::toSixDoFPose(pFrame->GetPose().Tcw(), C, A);
    baParams.param_push_back(C[0]), baParams.param_push_back(C[1]),
        baParams.param_push_back(C[2]);
    baParams.param_push_back(A[0]), baParams.param_push_back(A[1]),
        baParams.param_push_back(A[2]);
    baParams.viwIdx_push_back(pFrame->mnId, 0);
    baParams.viwFixedFlag_push_back(pFrame->mnId, pFrame->mnId == 0);
  }
  Vec2 cxcy(pFrame->cx, pFrame->cy);
  double f = pFrame->fx;

  int nInitialCorrespondences = 0;
  std::vector<int> vecIdxKpts;
  std::vector<ceres::ResidualBlockId> vecResBlkIds;

  ceres::Problem problem;
  for (int i = 0; i < pFrame->mNumOfKPts; i++)
  {
    MapPoint *pMP = pFrame->mvpMapPoints[i];
    if (pMP == NULL || pFrame->mvbOutlier[i])
      continue;

    if (pFrame->mvuRight[i] < 0)
    {
      Vec2 obs;
      const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
      obs << kpUn.pt.x, kpUn.pt.y;
      double invSigma = sqrt(pFrame->mvInvLevelSigma2[kpUn.octave]);
      Vec3 pt3D = Converter::toVector3d(pMP->GetWorldPos());

      ceres::CauchyLoss *loss =
          bRobust ? new ceres::CauchyLoss(Param::TH_CHI_2D) : NULL;
      ceres::CostFunction *costFun =
          ReprojectionErrorPoseOnly::create(obs - cxcy, pt3D, f, invSigma);
      ceres::ResidualBlockId bkId = problem.AddResidualBlock(
          costFun, loss, baParams.mutable_view(pFrame->mnId),
          baParams.mutable_view(pFrame->mnId) + 1,
          baParams.mutable_view(pFrame->mnId) + 2,
          baParams.mutable_view(pFrame->mnId) + 3,
          baParams.mutable_view(pFrame->mnId) + 4,
          baParams.mutable_view(pFrame->mnId) + 5);
      vecResBlkIds.push_back(bkId);
      vecIdxKpts.push_back(i);

      nInitialCorrespondences++;
    }
  }

  if (nInitialCorrespondences < 3)
  {
    std::cout << "Insufficient observations for pose estimation!" << std::endl;
    return 0;
  }

  ceres::Solver::Options options;
  setCeresOption(options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //-----------------------------------------------------------------------------------------------------

  // Refine
  //-----------------------------------------------------------------------------------------------------
  const double *const p = baParams.mutable_view(pFrame->mnId);
  Vec6 pose;
  for (int i = 0; i < 6; ++i)
    pose[i] = p[i];

  int nBad = 0;
  for (size_t i = 0; i < vecIdxKpts.size(); i++)
  {
    size_t idx = vecIdxKpts[i];
    Vec2 obs;
    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[idx];
    obs << kpUn.pt.x - pFrame->cx, kpUn.pt.y - pFrame->cy;

    MapPoint *pMP = pFrame->mvpMapPoints[idx];
    Vec3 pt3D = Converter::toVector3d(pMP->GetWorldPos());
    Vec2 res =
        ReprojectionErrorPoseOnly::back_project(obs, pt3D, pose, pFrame->fx);
    // std::cout << "res: " << res.norm() << std::endl;

    if (res.norm() > Param::TH_CHI_2D)
    {
      pFrame->mvbOutlier[idx] = true;
      ceres::ResidualBlockId blk = vecResBlkIds[i];
      problem.RemoveResidualBlock(blk);
      nBad++;
    }
    else
      pFrame->mvbOutlier[idx] = false;
  }
  if (nInitialCorrespondences - nBad < 5)
  {
    // std::cout << "Insufficient observations for pose estimation after
    // filter!" << std::endl;
    return 0;
  }

  // optimize again
  ceres::Solve(options, &problem, &summary);
  //-----------------------------------------------------------------------------------------------------

  // Recover the pose data
  Mat4 tmpT = Converter::toMatrixPose(baParams.mutable_view(pFrame->mnId));
  pFrame->SetPose(Converter::toCvMat(tmpT));

  return nInitialCorrespondences - nBad;
}

bool Optimizer::LocalBundleAdjustment(KeyFrame *pKF, FrameID baseFrameId,
                                      bool *pbStopFlag, Map *pMap)
{
  std::list<KeyFrame *> lLocalKeyFrames;
  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  std::vector<FrameID> vKeyViewIds;
  vKeyViewIds.push_back(pKF->mnId);

  pForceBAStopFlag = pbStopFlag;

  // const std::vector<KeyFrame *> vNeighKFs =
  // pKF->GetVectorCovisibleKeyFrames();
  const std::vector<KeyFrame *> vNeighKFs =
      pKF->GetBestCovisibilityKeyFrames(30);
  for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
  {
    KeyFrame *pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->mnId;
    if (!pKFi->isBad())
    {
      lLocalKeyFrames.push_back(pKFi);
      vKeyViewIds.push_back(pKFi->mnId);
    }
  }

  // Local MapPoints seen in Local KeyFrames
  std::list<MapPoint *> lLocalMapPoints;
  for (const auto &lit : lLocalKeyFrames)
  {
    std::vector<MapPoint *> vpMPs = lit->GetMapPointMatches();
    for (const auto &vit : vpMPs)
    {
      MapPoint *pMP = vit;
      if (pMP == NULL || pMP->isBad())
        continue;

      if (pMP->mnBALocalForKF != pKF->mnId)
      {
        lLocalMapPoints.push_back(pMP);
        pMP->mnBALocalForKF = pKF->mnId;
      }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  std::list<KeyFrame *> lFixedCameras;
  for (const auto &lit : lLocalMapPoints)
  {
    std::map<KeyFrame *, size_t> observations = lit->GetObservations();
    for (const auto &mit : observations)
    {
      KeyFrame *pKFi = mit.first;
      if (pKFi->mnBALocalForKF != pKF->mnId &&
          pKFi->mnBAFixedForKF != pKF->mnId)
      {
        pKFi->mnBAFixedForKF = pKF->mnId;
        if (pKFi->isBad())
          continue;

        lFixedCameras.push_back(pKFi);
        vKeyViewIds.push_back(pKFi->mnId);
      }
    }
  }

  BAParameters baParams;

  if (!setLBAParams(lLocalKeyFrames, lFixedCameras, lLocalMapPoints,
                    baseFrameId, baParams))
  {
    std::cout << "Failed to build LBA problem!" << std::endl;
    return false;
  }

  if (pbStopFlag && *pbStopFlag)
    return false;

  ceres::Problem problem;
  if (!buildLBAProblem(lLocalMapPoints, vKeyViewIds, baParams, problem,
                       Param::TH_CHI_2D)) // build for back-projection errors
    return false;

  // for GPS and IMU data
  if (nprameter.use == true)
    if (!buildXYProblem(lLocalKeyFrames, baParams, problem))
    {
      cout << "XY约束不成功" << endl;
      return false;
    }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_solver_time_in_seconds = Param::MAX_SLOVER_TIME;
  options.max_num_iterations = Param::MAX_ITERATION_NUMBER;
  size_t nthreads = std::thread::hardware_concurrency();
  options.num_threads = nthreads > Param::MAX_THREAD_FOR_SOLVER
                            ? Param::MAX_THREAD_FOR_SOLVER
                            : nthreads;

  DummyIterationCallback callback;
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!summary.IsSolutionUsable())
  {
    std::cout << "--Failed to run LBA!!" << std::endl;
    return false;
  }

  // check inlier obsevs
  vector<pair<KeyFrame *, MapPoint *>> vToErase;
  for (const auto &lit : lLocalMapPoints)
  {
    MapPoint *pMP = lit;
    if (pMP->isBad())
      continue;

    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
    for (const auto &mit : observations)
    {
      KeyFrame *pKF = mit.first;
      if (pKF->isBad())
        continue;
      if (pKF->mvuRight[mit.second] < 0) // for mono
      {
        const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit.second];
        double cx = pKF->cx, cy = pKF->cy;
        double f = pKF->fx;
        ReprojectionError errEq(kpUn.pt.x - cx, kpUn.pt.y - cy, f, 1.0);
        double res, depth;
        errEq.computeErrorAndDepth(baParams.mutable_view(pKF->mnId),
                                   baParams.mutable_point(pMP->mnId), res,
                                   depth);

        if (res > Param::TH_CHI_2D || depth < 0.0)
          vToErase.push_back(make_pair(pKF, pMP));
      }
    }
  }

  std::cout << "--LBA - " << summary.BriefReport() << std::endl;
  //   std::cout << "--LBA - " << summary.num_residual_blocks << std::endl;

  // Get Map Mutex
  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty())
  {
    for (size_t i = 0; i < vToErase.size(); i++)
    {
      KeyFrame *pKFi = vToErase[i].first;
      MapPoint *pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover Key Frames
  for (auto &lit : lLocalKeyFrames)
  {
    KeyFrame *pKF = lit;
    Mat4 tmpT = Converter::toMatrixPose(baParams.mutable_view(pKF->mnId));
    pKF->SetPose(camera::Pose(Converter::toCvMat(tmpT)));
  }

  // Points
  for (auto &lit : lLocalMapPoints)
  {
    MapPoint *pMP = lit;
    const double *pt = baParams.mutable_point(pMP->mnId);
    pMP->SetWorldPos(Converter::toCvMat(Vec3(pt[0], pt[1], pt[2])));
    pMP->UpdateNormalAndDepth();
  }
  return true;
}

// internal function
void setCeresOption(ceres::Solver::Options &options)
{
  // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  // options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  options.max_solver_time_in_seconds = Param::MAX_SLOVER_TIME;
  options.max_num_iterations = Param::MAX_ITERATION_NUMBER;
  size_t nthreads = std::thread::hardware_concurrency();
  options.num_threads = nthreads > Param::MAX_THREAD_FOR_SOLVER
                            ? Param::MAX_THREAD_FOR_SOLVER
                            : nthreads;
}

bool setLBAParams(const std::list<KeyFrame *> &lpKFs,
                  const std::list<KeyFrame *> &lFixedKFs,
                  const std::list<MapPoint *> &lpMP, FrameID baseFrameId,
                  BAParameters &baParams)
{
  if (lpKFs.size() < 1 || lpMP.size() < 20)
    return false;

  // Set Local KeyFrame
  size_t idx = 0;
  for (auto lit = lpKFs.begin(); lit != lpKFs.end(); lit++)
  {
    KeyFrame *pKF = *lit;
    Vec3 C, A;
    Converter::toSixDoFPose(pKF->GetPose().Tcw(), C, A);
    baParams.param_push_back(C[0]), baParams.param_push_back(C[1]),
        baParams.param_push_back(C[2]);
    baParams.param_push_back(A[0]), baParams.param_push_back(A[1]),
        baParams.param_push_back(A[2]);

    baParams.viwIdx_push_back(pKF->mnId, idx++);
    baParams.viwFixedFlag_push_back(pKF->mnId, pKF->mnId == baseFrameId);
  }
  // Set Fixed KeyFrame
  for (auto lit = lFixedKFs.begin(); lit != lFixedKFs.end(); lit++)
  {
    KeyFrame *pKFi = *lit;
    Vec3 C, A;
    Converter::toSixDoFPose(pKFi->GetPose().Tcw(), C, A);
    baParams.param_push_back(C[0]), baParams.param_push_back(C[1]),
        baParams.param_push_back(C[2]);
    baParams.param_push_back(A[0]), baParams.param_push_back(A[1]),
        baParams.param_push_back(A[2]);
    baParams.viwIdx_push_back(pKFi->mnId, idx++);
    baParams.viwFixedFlag_push_back(pKFi->mnId, true);
  }

  // Fill 3D map points
  idx = 0;
  for (auto lit = lpMP.begin(); lit != lpMP.end(); lit++)
  {
    MapPoint *pMP = *lit;

    Vec3 pt3D = Converter::toVector3d(pMP->GetWorldPos());
    baParams.param_push_back(pt3D[0]), baParams.param_push_back(pt3D[1]),
        baParams.param_push_back(pt3D[2]);
    baParams.ptsIdx_push_back(pMP->mnId, idx++);
  }

  return true;
}

bool setCam2IMU(const cv::Mat &Tic, const cv::Mat &Ric,
                BAParameters &baParams)
{
  if (Ric.empty() || Tic.empty())
    return false;

  Vec3 vTic = Converter::toVector3d(Tic);
  baParams.param_push_back(vTic[0]), baParams.param_push_back(vTic[1]),
      baParams.param_push_back(vTic[2]);

  Mat3 matRic = Converter::toMatrix3d(Ric);
  Vec3 angleAxis;
  ceres::RotationMatrixToAngleAxis((double *)matRic.data(), angleAxis.data());
  baParams.param_push_back(angleAxis[0]),
      baParams.param_push_back(angleAxis[1]),
      baParams.param_push_back(angleAxis[2]);
  baParams.view2IMUIdx_push_back(0, 0);

  return true;
}

// build ceres problem
bool buildLBAProblem(const std::list<MapPoint *> &lLocalMapPoints,
                     const std::vector<FrameID> &vKeyViewIds,
                     BAParameters &baParams, ceres::Problem &problem,
                     float th)
{
  if (lLocalMapPoints.empty() || vKeyViewIds.empty())
    return false;

  for (auto lit = lLocalMapPoints.begin(); lit != lLocalMapPoints.end();
       lit++)
  {
    MapPoint *pMP = *lit;
    if (pMP->isBad())
      continue;

    const map<KeyFrame *, size_t> observations = pMP->GetObservations();

    for (auto mit = observations.begin(); mit != observations.end(); mit++)
    {
      KeyFrame *pKF = mit->first;
      if (pKF->isBad())
        continue;

      const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
      if (pKF->mvuRight[mit->second] < 0)
      {
        float cx = pKF->cx, cy = pKF->cy;
        //         float f = (pKF->fx + pKF->fy) / 2.0;
        float f = pKF->fx;
        float invSigma = sqrt(pKF->mvInvLevelSigma2[kpUn.octave]);
        ceres::HuberLoss *loss = new ceres::HuberLoss(th);
        // ceres::CauchyLoss* loss = new ceres::CauchyLoss(th);

        ceres::CostFunction *costFun = ReprojectionError::create(
            kpUn.pt.x - cx, kpUn.pt.y - cy, f, invSigma);

        problem.AddResidualBlock(costFun, loss,
                                 baParams.mutable_view(pKF->mnId),
                                 baParams.mutable_view(pKF->mnId) + 1,
                                 baParams.mutable_view(pKF->mnId) + 2,
                                 baParams.mutable_view(pKF->mnId) + 3,
                                 baParams.mutable_view(pKF->mnId) + 4,
                                 baParams.mutable_view(pKF->mnId) + 5,
                                 baParams.mutable_point(pMP->mnId));
      }
    }
  }
  // set fixed constants
  for (size_t i = 0; i < vKeyViewIds.size(); ++i)
  {
    ViewID viwId = vKeyViewIds[i];
    if (baParams.view_fixed(viwId))
    {
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId));
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId) + 1);
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId) + 2);
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId) + 3);
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId) + 4);
      problem.SetParameterBlockConstant(baParams.mutable_view(viwId) + 5);
    }
  }
  return true;
}

// here we ignore the fixed frames
bool buildXYProblem(std::list<KeyFrame *> &lLocalKeyFrames,
                    BAParameters &baParams, ceres::Problem &problem)
{
  // std::vector<KeyFrame *> vecKeyFrames;
  // vecKeyFrames.assign(lLocalKeyFrames.begin(), lLocalKeyFrames.end());
  std::cout << "start to build XY problem" << std::endl;
  std::cout << "Number of key frame: " << lLocalKeyFrames.size() << std::endl;

  if (lLocalKeyFrames.size() < 3)
    return false;

  // 将位置信息存到全局变量中
  // Eigen::Matrix<double, 3, Eigen::Dynamic> ref(3, nprameter.ref.size()); // 相对
  // Eigen::Matrix<double, 3, Eigen::Dynamic> abs(3, nprameter.abs.size()); // 绝对

  // int num_kf = 0;
  // for (int i = 0; i < vecKeyFrames.size(); i++)
  for (auto lit = lLocalKeyFrames.begin(); lit != lLocalKeyFrames.end(); lit++)
  {
    KeyFrame *pKF = *lit;
    // KeyFrame *pKF = vecKeyFrames[i];
    // camera::AbsXY abs_xy = pKF->GetAbsXY();
    const camera::AbsXY &abs_xy = pKF->GetAbsXY();
    if (!abs_xy.flag)
    {
      continue; // 此帧的置信度太低
    }

    // 之前是关键帧的图片和绝对位姿同时传进来，现在改为分开传进来
    Eigen::Vector3d abs_p = abs_xy.xyz;
    nprameter.abs.push_back(abs_p);

    ViewID viwId = pKF->mnId;
    Vec3 ref_p(*(baParams.mutable_view(viwId)),
               *(baParams.mutable_view(viwId) + 1),
               *(baParams.mutable_view(viwId) + 2));

    // 判断当前关键帧是否符合全局定位要求，如果是则插入
    // auto it = find(nprameter.kfIDs.begin(), nprameter.kfIDs.end(), pKF->mnId);
    // if (it != nprameter.kfIDs.end())
    // {
    //   num_kf++;
    //   int idx = std::distance(nprameter.kfIDs.begin(), it);
    //   auto abs_p = nprameter.kfPos[idx];

    //   abs_xy.xyz = nprameter.kfPos[idx];
    //   abs_xy.qwxyz = nprameter.kfOrin[idx];
    //   abs_xy.sqrt_cov = Mat3::Identity() * nprameter.kfConf[idx];

    //   // 插入绝对
    //   std::cout << "\n\n插入关键帧：" << pKF->mnId << "\n\n";
    //   nprameter.abs.push_back(abs_p);
    //   vecKeyFrames[i]->SetAbsXY(abs_xy);
    //   // 验证
    //   std::cout << "\n\n更新后的绝对位置：";
    //   std::cout << vecKeyFrames[i]->GetAbsXY().xyz;
    //   std::cout << "\n\n";

    // 插入相对
    nprameter.ref.push_back(ref_p);
    // }
  }
  Eigen::Matrix<double, 3, Eigen::Dynamic> ref(3, nprameter.ref.size()); // 相对
  Eigen::Matrix<double, 3, Eigen::Dynamic> abs(3, nprameter.abs.size()); // 绝对
  // if (num_kf < 3)
  // {
  //   std::cout << "筛选后剩下" << num_kf << "张关键帧，跳过！\n";
  //   return false;
  // }
  // std::cout << "Checkpoint 1\n\n";
  // std::cout << ref.size() << " " << abs.size() << " " << nprameter.ref.size() << "\n\n";
  for (int i = 0; i < nprameter.abs.size(); i++)
  {
    ref(0, i) = nprameter.ref[i](0);
    ref(1, i) = nprameter.ref[i](1);
    ref(2, i) = nprameter.ref[i](2);

    abs(0, i) = nprameter.abs[i](0);
    abs(1, i) = nprameter.abs[i](1);
    abs(2, i) = nprameter.abs[i](2);
  }
  // 求解两个轨迹之间的RT矩阵
  Eigen::Matrix4d rt = Eigen::umeyama(ref, abs, true);
  Eigen::Vector3d t;
  auto R = rt.block(0, 0, 3, 3);
  t << rt(0, 3), rt(1, 3), rt(2, 3);

  // std::cout << "Checkpoint 2\n\n";

  // for (int idx = 0; idx < vecKeyFrames.size(); idx++)
  for (auto lit = lLocalKeyFrames.begin(); lit != lLocalKeyFrames.end(); lit++)
  {
    KeyFrame *pKF = *lit;
    // KeyFrame *pKF = vecKeyFrames[idx];
    // 判断是否要跳过
    // auto it = find(nprameter.kfIDs.begin(), nprameter.kfIDs.end(), pKF->mnId);
    // if (it == nprameter.kfIDs.end())
    // {
    //   std::cout << "该关键帧不符合全局定位要求 ...\n";
    //   continue;
    // }
    // const camera::AbsXY abs_xy = pKF->GetAbsXY();
    const camera::AbsXY &abs_xy = pKF->GetAbsXY();

    Eigen::Vector3d abs_p = abs_xy.xyz;

    ViewID viwId = pKF->mnId;
    Vec3 ref_p(*(baParams.mutable_view(viwId)),
               *(baParams.mutable_view(viwId) + 1),
               *(baParams.mutable_view(viwId) + 2));

    ref_p = R * ref_p;
    ref_p = ref_p + t;

    Vec3 dis = abs_p - ref_p;
    std::cout << "Initial xyz:" << ref_p.transpose() << std::endl;
    std::cout << "absolute xyz:" << abs_p.transpose() << std::endl;
    std::cout << "residual:" << dis.norm() << " "
              << "m" << std::endl;

    // 判断是否要进入BA
    if (dis.norm() > 10)
    {
      std::cout << "Unrealiable absolute XYZ observation!" << std::endl;
      continue;
    }

    ceres::CostFunction *cf = PositionXYError::create(abs_p, abs_xy.sqrt_cov, R, t, 1.0);
    ceres::HuberLoss *loss = new ceres::HuberLoss(Param::TH_CHI_3D);
    problem.AddResidualBlock(cf, loss, baParams.mutable_view(pKF->mnId),
                             baParams.mutable_view(pKF->mnId) + 1,
                             baParams.mutable_view(pKF->mnId) + 2);
  }

  return true;
}
