#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <mutex>

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "Tracking.h"

namespace ORB_SLAM2 {

class Tracking;
class Map;

class LocalMapping {
 public:
  LocalMapping(Map* pMap, const float bMonocular);

  void SetTracker(Tracking* pTracker);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame* pKF);

  // Thread Synch
  void RequestStop();
  void RequestReset();
  bool isReseted();

  bool Stop();
  void Release();
  bool isStopped();
  bool stopRequested();
  bool AcceptKeyFrames();
  void SetAcceptKeyFrames(bool flag);
  bool SetNotStop(bool flag);

  void InterruptBA();

  void RequestFinish();
  bool isFinished();

  int KeyframesInQueue() {
    unique_lock<std::mutex> lock(mMutexNewKFs);
    return mlNewKeyFrames.size();
  }

 protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();

  void MapPointCulling();
  void SearchInNeighbors();

  void KeyFrameCulling();

  cv::Mat ComputeF12(KeyFrame*& pKF1, KeyFrame*& pKF2);

  cv::Mat SkewSymmetricMatrix(const cv::Mat& v);

  bool mbMonocular;

  void ResetIfRequested();
  bool mbResetRequested;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Map* mpMap;

  Tracking* mpTracker;

  std::list<KeyFrame*> mlNewKeyFrames;

  KeyFrame* mpCurrentKeyFrame;

  std::list<MapPoint*> mlpRecentAddedMapPoints;

  std::mutex mMutexNewKFs;

  bool mbAbortBA;

  bool mbStopped;
  bool mbStopRequested;
  bool mbNotStop;
  std::mutex mMutexStop;

  bool mbAcceptKeyFrames;
  std::mutex mMutexAccept;
};

}  // namespace ORB_SLAM2

#endif  // LOCALMAPPING_H
