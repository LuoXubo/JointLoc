#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <mutex>
#include <opencv2/core/core.hpp>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace ORB_SLAM2 {

class KeyFrame;
class Map;
class Frame;

class MapPoint {
 public:
  MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap);
  MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame, const int& idxF);

  void SetWorldPos(const cv::Mat& Pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  float Depth(const cv::Mat& R, const cv::Mat& C);

  KeyFrame* GetReferenceKeyFrame();

  std::map<KeyFrame*, size_t> GetObservations();
  int Observations();

  void AddObservation(KeyFrame* pKF, size_t idx);
  void EraseObservation(KeyFrame* pKF);

  int GetIndexInKeyFrame(KeyFrame* pKF);
  bool IsInKeyFrame(KeyFrame* pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint* pMP);
  MapPoint* GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  float GetDistance();
  int PredictScale(const float& currentDist, KeyFrame* pKF);
  int PredictScale(const float& currentDist, Frame* pF);

 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;
  bool mbTrackInView;
  int mnTrackScaleLevel;
  float mTrackViewCos;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;

  static std::mutex mGlobalMutex;

 protected:
  // Position in absolute coordinates
  cv::Mat mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame*, size_t> mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame* mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint* mpReplaced;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  float mfDistance;

  Map* mpMap;

  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
};

}  // namespace ORB_SLAM2

#endif  // MAPPOINT_H
