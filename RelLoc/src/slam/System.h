
#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM2
{
  class Viewer;
  class FrameDrawer;
  class Map;
  class Tracking;
  class LocalMapping;
  class LoopClosing;

  class System
  {
  public:
    // Input sensor
    enum eSensor
    {
      MONOCULAR = 0,
      STEREO = 1,
      RGBD = 2
    };

  public:
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
    // Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile,
           const eSensor sensor, bool bUseViewer);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
    // grayscale. Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp,
                           const camera::AbsXY &abs_xy);

    // This stops local mapping thread (map building) and performs only camera
    // tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at:
    // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename,
                             const vector<double> &vecTimeStamp);

    void SaveMapPoints(const string &filename);

    void SaveTrajectory(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint *> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    static bool exportToPly(const std::string &sFileName,
                            const std::vector<Vec3> &vecPts,
                            const vector<Vec3> &vecColors);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
    // grayscale. Returns the camera pose (empty if tracking fails). cv::Mat
    // TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double
    // &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
    // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
    // if tracking fails). cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat
    // &depthmap, const double &timestamp);

    bool HasSuffix(const std::string &str, const std::string &suffix);

  private:
    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary *mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop
    // detection).
    KeyFrameDatabase *mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map *mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints
    // and performs relocalization if tracking fails.
    Tracking *mpTracker;

    // Local Mapper. It manages the local map and performs local bundle
    // adjustment.
    LocalMapping *mpLocalMapper;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the
    // System object.
    std::thread *mptLocalMapping;
    std::thread *mptLoopClosing;

    // // 监听线程，负责接收
    // getAbs *mpGetAbs;
    // std::thread *mptGetAbs;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer *mpViewer;
    FrameDrawer *mpFrameDrawer;
    MapDrawer *mpMapDrawer;

    std::thread *mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint *> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::map<double, cv::Mat> mSLAMPoses;
    std::mutex mMutexState;
  };

} // namespace ORB_SLAM2

#endif // SYSTEM_H
