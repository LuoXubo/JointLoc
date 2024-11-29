#include "System.h"

#ifdef SHOW_WITH_PANGOLIN
#include <pangolin/pangolin.h>
#endif

#include <unistd.h>

#include <3rdParty/stlplus3/filesystemSimplified/file_system.hpp>
#include <iomanip>
#include <thread>

#include "Config.h"
#include "Converter.h"

extern NormalPrameter nprameter;

namespace ORB_SLAM2
{
  using namespace BASE;

  System::System(const string &strVocFile, const string &strSettingsFile,
                 const eSensor sensor, bool bUseViewer)
      : mSensor(sensor), mbReset(false), mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
  {
    std::cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
      std::cout << "Monocular" << std::endl;
    else if (mSensor == STEREO)
      std::cout << "Stereo" << std::endl;
    else if (mSensor == RGBD)
      std::cout << "RGB-D" << std::endl;

    // Check settings file
    std::cout << "strSetting: " << strSettingsFile << std::endl;

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      std::cerr << "Failed to open settings file at: " << strSettingsFile
                << std::endl;
      exit(-1);
    }

    // Load ORB Vocabulary
    clock_t tStart = clock();
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (!stlplus::file_exists(strVocFile))
    {
      std::cerr << "No vocabulary file !" << std::endl;
      exit(-1);
    }
    if (HasSuffix(strVocFile, ".txt"))
      bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
      bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    std::cout << "VOC load time: " << double(clock() - tStart) / CLOCKS_PER_SEC
              << std::endl;

    if (!bVocLoad)
    {
      std::cerr << "Wrong path to vocabulary. " << std::endl;
      std::cerr << "Falied to open at: " << strVocFile << std::endl;
      exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl;

    // Create the KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Map
    mpMap = new Map();

    // Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // Initialize the Tracking thread
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

    // Initialize the Viewer thread and launch
    if (bUseViewer)
    {
      mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,
                            strSettingsFile);
      mptViewer = new thread(&Viewer::Run, mpViewer);
      mpTracker->SetViewer(mpViewer);
    }
    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);

    mpLocalMapper->SetTracker(mpTracker);

    std::cout << "Waitting for images..." << std::endl;
  }

  cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                 const camera::AbsXY &abs_xy)
  {
    if (mSensor != MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
              "Monocular."
           << endl;
      exit(-1);
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }
    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
      }
    }

    cv::Mat Twc;
    mpTracker->GrabImageMonocular(im, timestamp, abs_xy, Twc);
    //   if (!mpTracker->GrabImageMonocular(im, timestamp, abs_xy, Twc) &&
    //       mpTracker->mState == Tracking::LOST) {
    //     unique_lock<mutex> lock(mMutexReset);
    //     mbReset = true;
    //     mpTracker->Reset();
    //     mbReset = false;
    //   }
    //   if (Twc.rows == 4 && Twc.cols == 4)
    //     mSLAMPoses.emplace(timestamp, Twc);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Twc;
  }

  /*
  cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const
  double &timestamp)
  {
      if(mSensor!=STEREO)
      {
          cerr << "ERROR: you called TrackStereo but input sensor was not set to
  STEREO." << endl; exit(-1);
      }

      // Check mode change
      {
          unique_lock<mutex> lock(mMutexMode);
          if(mbActivateLocalizationMode)
          {
              mpLocalMapper->RequestStop();

              // Wait until Local Mapping has effectively stopped
              while(!mpLocalMapper->isStopped())
              {
                  usleep(1000);
              }

              mpTracker->InformOnlyTracking(true);
              mbActivateLocalizationMode = false;
          }
          if(mbDeactivateLocalizationMode)
          {
              mpTracker->InformOnlyTracking(false);
              mpLocalMapper->Release();
              mbDeactivateLocalizationMode = false;
          }
      }

      // Check reset
      {
          unique_lock<mutex> lock(mMutexReset);
          if(mbReset)
          {
              mpTracker->Reset();
              mbReset = false;
          }
      }

      cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

      unique_lock<mutex> lock2(mMutexState);
      mTrackingState = mpTracker->mState;
      mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
      mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
      return Tcw;
  }

  cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const
  double &timestamp)
  {
      if(mSensor!=RGBD)
      {
          cerr << "ERROR: you called TrackRGBD but input sensor was not set to
  RGBD." << endl; exit(-1);
      }

      // Check mode change
      {
          unique_lock<mutex> lock(mMutexMode);
          if(mbActivateLocalizationMode)
          {
              mpLocalMapper->RequestStop();

              // Wait until Local Mapping has effectively stopped
              while(!mpLocalMapper->isStopped())
              {
                  usleep(1000);
              }

              mpTracker->InformOnlyTracking(true);
              mbActivateLocalizationMode = false;
          }
          if(mbDeactivateLocalizationMode)
          {
              mpTracker->InformOnlyTracking(false);
              mpLocalMapper->Release();
              mbDeactivateLocalizationMode = false;
          }
      }

      // Check reset
      {
          unique_lock<mutex> lock(mMutexReset);
          if(mbReset)
          {
              mpTracker->Reset();
              mbReset = false;
          }
      }

      cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

      unique_lock<mutex> lock2(mMutexState);
      mTrackingState = mpTracker->mState;
      mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
      mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
      return Tcw;
  }
  */

  void System::ActivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
  }

  void System::DeactivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
  }

  bool System::MapChanged()
  {
    static int n = 0;
    int curn = mpMap->GetLastBigChangeIdx();
    if (n < curn)
    {
      n = curn;
      return true;
    }
    else
      return false;
  }

  void System::Reset()
  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }

  void System::Shutdown()
  {
    mpLocalMapper->RequestFinish();

#ifdef RUN_WITH_LOOPCLOSURE
    mpLoopCloser->RequestFinish();
#endif

    if (mpViewer)
    {
      mpViewer->RequestFinish();
      while (!mpViewer->isFinished())
        usleep(1000);
    }

    // Wait until all thread have effectively stopped

    bool b1 = false, b2 = false;
#ifdef RUN_WITH_LOOPCLOSURE
    b1 = !mpLoopCloser->isFinished();
    b2 = mpLoopCloser->isRunningGBA();
#endif

    while (!mpLocalMapper->isFinished() || b1 || b2)
    {
      usleep(1000);
    }

#ifdef SHOW_WITH_PANGOLIN
    if (mpViewer)
      pangolin::BindToContext("ORB-SLAM2: Map Viewer");
#endif
  }

  void System::SaveTrajectoryTUM(const string &filename)
  {
    cout << "Saving camera trajectory (TUM) to " << filename << " ..." << endl;
    //   if (mSensor == MONOCULAR) {
    //     cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
    //     return;
    //   }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPose().Twc();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized
    // by BA and pose graph). We need to get first the keyframe pose and then
    // concatenate the relative transformation. Frames not localized (tracking
    // failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
    // a flag which is true when tracking failed (lbL).
    std::list<ORB_SLAM2::KeyFrame *>::iterator lRit =
        mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (std::list<cv::Mat>::iterator
             lit = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++)
    {
      if (*lbL)
        continue;

      KeyFrame *pKF = *lRit;

      cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

      // If the reference keyframe was culled, traverse the spanning tree to get a
      // suitable keyframe.
      while (pKF->isBad())
      {
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose().Tcw() * Two;

      cv::Mat Tcw = (*lit) * Trw;
      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

      // Local to world frame projection
      Eigen::Vector3d pos;
      pos << twc.at<float>(0), twc.at<float>(1), twc.at<float>(2);
      pos = nprameter.l2g_rotation * pos + nprameter.l2g_translation;

      Eigen::Quaterniond q = Converter::toQuaternion(Rwc);

      f << setprecision(6) << *lT << " " << setprecision(9) << pos(0)
        << " " << pos(1) << " " << pos(2) << " " << q.w()
        << " " << q.x() << " " << q.y() << " " << q.z() << endl;
      // f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0)
      //   << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q.w()
      //   << " " << q.x() << " " << q.y() << " " << q.z() << endl;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  void System::SaveKeyFrameTrajectoryTUM(const string &filename)
  {
    cout << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];

      // pKF->SetPose(pKF->GetPose()*Two);

      if (pKF->isBad())
        continue;

      cv::Mat R = pKF->GetPose().Rcw().t();

      Eigen::Quaterniond q = Converter::toQuaternion(R);

      cv::Mat t = pKF->GetPose().Ow();

      // Local to world frame projection
      Eigen::Vector3d pos;
      pos << t.at<float>(0), t.at<float>(1), t.at<float>(2);
      pos = nprameter.l2g_rotation * pos + nprameter.l2g_translation;

      f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
        << pos(0) << " " << pos(1) << " " << pos(2) << " "
        << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;

      // f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
      //   << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
      //   << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << endl;
    }

    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  void System::SaveTrajectoryKITTI(const string &filename,
                                   const vector<double> &vecTimeStamp)
  {
    cout << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
      cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
      return;
    }

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPose().Twc();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized
    // by BA and pose graph). We need to get first the keyframe pose and then
    // concatenate the relative transformation. Frames not localized (tracking
    // failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
    // a flag which is true when tracking failed (lbL).
    int idx = 0;
    list<ORB_SLAM2::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                 lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++)
    {
      ORB_SLAM2::KeyFrame *pKF = *lRit;

      cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

      while (pKF->isBad())
      {
        //  cout << "bad parent" << endl;
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose().Tcw() * Two;

      cv::Mat Tcw = (*lit) * Trw;
      cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
      cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

      f << setprecision(9) << vecTimeStamp[idx] << " " << Rwc.at<float>(0, 0)
        << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " "
        << twc.at<float>(0) << " " << Rwc.at<float>(1, 0) << " "
        << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " "
        << twc.at<float>(1) << " " << Rwc.at<float>(2, 0) << " "
        << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " "
        << twc.at<float>(2) << endl;
      idx++;
    }
    f.close();
    cout << endl
         << "trajectory saved!" << endl;
  }

  void System::SaveMapPoints(const string &filename)
  {
    std::vector<Vec3> vecMapPts, vecColors;

    std::cout << "Saving map points to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    for (size_t i = 0; i < vpKFs.size(); ++i)
    {
      cv::Mat twc = vpKFs[i]->GetPose().Ow();
      vecMapPts.push_back(
          Vec3(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2)));

      // vecMapPts.push_back(twc);
      Vec3 color = Vec3::Zero();
      color[0] = 255, color[1] = 0, color[2] = 0;
      vecColors.push_back(color);

      // for abs position
      if (vpKFs[i]->GetAbsXY().flag)
      {
        camera::AbsXY abs_xy = vpKFs[i]->GetAbsXY();
        const double z = vpKFs[i]->GetPose().Ow().at<float>(2);
        vecMapPts.push_back(abs_xy.xyz);
        color[0] = 0, color[1] = 255, color[2] = 0;
        vecColors.push_back(color);
      }
    }

    std::cout << "save map points: " << mpMap->GetAllMapPoints().size()
              << std::endl;

    const vector<MapPoint *> vecMapPoints = mpMap->GetAllMapPoints();
    for (size_t i = 0; i < vecMapPoints.size(); ++i)
    {
      if (vecMapPoints[i]->GetDistance() > 300.)
        continue;
      cv::Mat pos = vecMapPoints[i]->GetWorldPos();
      Vec3 vPts = Converter::toVector3d(vecMapPoints[i]->GetWorldPos());
      vecMapPts.push_back(vPts);
      Vec3 color = Vec3::Zero();
      color[0] = 255, color[1] = 255, color[2] = 255;
      vecColors.push_back(color);
    }

    exportToPly(filename, vecMapPts, vecColors);
  }

  int System::GetTrackingState()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
  }

  vector<MapPoint *> System::GetTrackedMapPoints()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
  }

  vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
  }

  bool System::exportToPly(const std::string &sFileName,
                           const std::vector<Vec3> &vecPts,
                           const std::vector<Vec3> &vecColors)
  {
    std::ofstream outfile;
    outfile.open(sFileName.c_str(), std::ios_base::out);

    outfile << "ply\n"
            << "format ascii 1.0\n"
            << "element vertex " << vecPts.size() << '\n'
            << "property float x\n"
            << "property float y\n"
            << "property float z\n"
            << "property uchar red\n"
            << "property uchar green\n"
            << "property uchar blue\n"
            << "end_header" << std::endl;

    for (size_t i = 0; i < vecPts.size(); ++i)
    {
      if (vecColors.size() < 1)
      {
        const Vec3 &pt = vecPts[i];
        float X = pt(0), Y = pt(1), Z = pt(2);

        outfile << std::fixed << std::setprecision(6) << X << " " << Y << " " << Z
                << " 255 255 255\n";
      }
      else
      {
        const Vec3 &pt = vecPts[i];
        float X = pt(0), Y = pt(1), Z = pt(2);

        const Vec3 &cl = vecColors[i];
        int b = cl[0], g = cl[1], r = cl[2];
        outfile << std::fixed << std::setprecision(6) << X << " " << Y << " " << Z
                << " " << std::fixed << std::setprecision(0) << b << " " << g
                << " " << r << "\n";
      }
    }
    outfile.flush();
    bool bOk = outfile.good();
    outfile.close();
    return bOk;
  }

  void System::SaveTrajectory(const string &filename)
  {
    cout << "Saving keyframe trajectory to " << filename << " ..." << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    // std::cout<< "Poses: " << mSLAMPoses.size() << std::endl;
    for (const auto &it : mSLAMPoses)
    {
      double timestamp = it.first;
      const cv::Mat &Tcw = it.second;
      f << setprecision(9) << timestamp << " " << Tcw.at<float>(0, 0) << " "
        << Tcw.at<float>(0, 1) << " " << Tcw.at<float>(0, 2) << " "
        << Tcw.at<float>(0, 3) << " " << Tcw.at<float>(1, 0) << " "
        << Tcw.at<float>(1, 1) << " " << Tcw.at<float>(1, 2) << " "
        << Tcw.at<float>(1, 3) << " " << Tcw.at<float>(2, 0) << " "
        << Tcw.at<float>(2, 1) << " " << Tcw.at<float>(2, 2) << " "
        << Tcw.at<float>(2, 3) << endl;
    }
    f.flush();
    f.close();

    cout << "Trajectory saved!" << endl;
  }

  bool System::HasSuffix(const std::string &str, const std::string &suffix)
  {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
  }

} // namespace ORB_SLAM2
