#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <pangolin/pangolin.h>

#include <mutex>

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

class MapDrawer {
 public:
  MapDrawer(Map *pMap, const string &strSettingPath);

  Map *mpMap;

  void DrawMapPoints();
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
  void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
  void SetCurrentCameraPose(const cv::Mat &Tcw);
  void SetCurrentGPSIMUPose(const cv::Mat &Tcw);
  void SetReferenceKeyFrame(KeyFrame *pKF);
  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
  void GetCurrentOpenGLGPSIMUMatrix(pangolin::OpenGlMatrix &M);

 private:
  float mKeyFrameSize;
  float mKeyFrameLineWidth;
  float mGraphLineWidth;
  float mPointSize;
  float mCameraSize;
  float mCameraLineWidth;

  cv::Mat mCameraPose;

  cv::Mat mGPSIMUPose;

  std::mutex mMutexCamera;
};

}  // namespace ORB_SLAM2

#endif  // MAPDRAWER_H
