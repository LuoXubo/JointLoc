#include "Tracking.h"

#include <iostream>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <unistd.h>

#include "Converter.h"
#include "ModelBA.h"
#include "ORBmatcher.h"
#include "PnPsolver.h"
#include "Parameters.h"
#include "loadArg.h"
#include "ViewPoint.h"
extern NormalPrameter nprameter;

using namespace std;

namespace ORB_SLAM2 {
using namespace BASE;

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB,
                   const string &strSettingPath, const int sensor)
    : mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false),
      mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
      mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys),
      mpMap(pMap), mnLastRelocFrameId(0), mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer) {
  cv::Mat matK, matDistCoef;
  SLAMConfig slamParam;
  loadArg::loadSLAMSetting(strSettingPath, matK, matDistCoef, slamParam);

  matK.copyTo(mK);
  matDistCoef.copyTo(mDistCoef);

  // Max/Min Frames to insert keyframes
  mMinFrames = 0;
  mMaxFrames = slamParam.settingParam.fps;
  mbf = slamParam.settingParam.bf;
  mbRGB = slamParam.settingParam.bRGB;

  // orb detector
  mpORBextractorLeft = new ORBextractor(
      slamParam.orbFeatParam.numFeatures, slamParam.orbFeatParam.scaleFactor,
      slamParam.orbFeatParam.numLevels, slamParam.orbFeatParam.numIniThFAST,
      slamParam.orbFeatParam.numMinThFAST);

  if (sensor == System::STEREO)
    mpORBextractorRight = new ORBextractor(
        slamParam.orbFeatParam.numFeatures, slamParam.orbFeatParam.scaleFactor,
        slamParam.orbFeatParam.numLevels, slamParam.orbFeatParam.numIniThFAST,
        slamParam.orbFeatParam.numMinThFAST);

  if (sensor == System::MONOCULAR)
    mpIniORBextractor = new ORBextractor(2 * slamParam.orbFeatParam.numFeatures,
                                         slamParam.orbFeatParam.scaleFactor,
                                         slamParam.orbFeatParam.numLevels,
                                         slamParam.orbFeatParam.numIniThFAST,
                                         slamParam.orbFeatParam.numMinThFAST);

  if (sensor == System::STEREO || sensor == System::RGBD)
    mThDepth = slamParam.settingParam.thDepth;

  if (sensor == System::RGBD)
    mDepthMapFactor = slamParam.settingParam.depthMapFactor;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetViewer(Viewer *pViewer) { mpViewer = pViewer; }

bool Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp,
                                  const camera::AbsXY &abs_xy, cv::Mat &Twc) {
  mImColor = im.clone();
  mImGray = im;

  if (mImGray.channels() == 3) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGR2GRAY);
  } else if (mImGray.channels() == 4) {
    if (mbRGB)
      cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    else
      cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
  }

  // clock_t startTime = clock();
  if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
    mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor,
                          mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
  else
    mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft,
                          mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

  // Set absolute xy measure
  mCurrentFrame.SetAbsXY(abs_xy);

  // Track tie points
  bool ret = Track(timestamp); 
  cv::Mat Ow = mCurrentFrame.GetPose().Ow().clone();
  nprameter.wait=nprameter.wait - 1;
//   cout << "相机坐标：" << mCurrentFrame.GetPose().tcw() <<endl;

//   if (ret)
//   {
//     if(nprameter.NP_Ready == true && nprameter.wait<1)
//         {
//             if(nprameter.Num_Ready<nprameter.Num_Need){
//                 Vec3 p_slam;
//                 cv2eigen(Ow, p_slam);
//                 nprameter.pose_slam.push_back(p_slam);
//                 
//                 Vec3 p_ref = abs_xy.xyz;
//                 nprameter.pose_ref.push_back(p_ref);
//                 
//                 nprameter.Num_Ready = nprameter.Num_Ready + 1;
// //                 nprameter.wait = 10;
//             }else
//             {
//                 cout << "准备估计归一化参数!!" <<endl;
//                 Normal_estimate();
//                 nprameter.NP_Ready = false;
//                 nprameter.use = true;
//             }
//                 
//         }
//   }
  return ret;
}

// bool Tracking::Track(const double &timestamp) {
//   if (mState == NO_IMAGES_YET)
//         mState = NOT_INITIALIZED;
//   mLastProcessedState = mState;
// 
//   unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
//   if (mState == NOT_INITIALIZED) {
//       if (mSensor == System::STEREO || mSensor == System::RGBD)
//           StereoInitialization();
//       else
//           MonocularInitialization();
//     mpFrameDrawer->Update(this);
//     
//     if (mState != OK) {
//       std::cout << "Not initilized!" << std::endl;
//       return false;
//     }
//   } else 
//   {
//     if (mbOnlyTracking)
//       return false;
// 
//     // std::cout << "--Tracking State: " << mState << " ";
//     bool bOK;
//     if (mState == OK) {
//       CheckReplacedInLastFrame();
//       if (!m_posePredictor.isValid() || mCurrentFrame.mnId < 2) {
//         std::cout << "Track with reference frame!" << std::endl;
//         bOK = TrackReferenceKeyFrame();
//         // initlize the predictor
//         m_posePredictor.initial(mInitialFrame.GetPose(), mLastFrame.GetPose());
//       } else {
//         // std::cout << "Track with motion model frame!" << std::endl;
//         bOK = TrackWithMotionModel(timestamp);
//         if (!bOK) {
//           std::cout << "\nFailed to track with MotionModel and then track with "
//                        "reference frame!"
//                     << std::endl;
//           bOK = TrackReferenceKeyFrame();
//         }
//       }
//     } else {
//       std::cout << "Failed to track!!" << std::endl;
//       return false;
//     }
//     mCurrentFrame.mpReferenceKF = mpReferenceKF;
// 
//     // If we have an initial estimation of the camera pose and matching. Track
//     // the local map.
//     if (bOK)
//       bOK = TrackLocalMap();
// 
//     if (bOK)
//       mState = OK;
//     else
//       mState = LOST;
// 
//     // Update drawer
//     mpFrameDrawer->Update(this);
// 
//     if (!bOK) {
//       std::cout << "\nThe tracking is bad, RESET the system!!\n";
//       return false;
//     }
//     // Update motion model
//     if (!mLastFrame.GetPose().Tcw().empty())
//       m_posePredictor.update(mCurrentFrame.GetPose());
//     else
//       m_posePredictor.initial(mCurrentFrame.GetPose(), mCurrentFrame.GetPose());
// 
//     // Clean VO matches
//     for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
//       MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
//       if (pMP && pMP->Observations() < 1) {
//         mCurrentFrame.mvbOutlier[i] = false;
//         mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
//       }
//     }
// 
//     // Delete temporal MapPoints
//     for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(),
//                                     lend = mlpTemporalPoints.end();
//          lit != lend; lit++) {
//       MapPoint *pMP = *lit;
//       delete pMP;
//     }
//     mlpTemporalPoints.clear();
// 
//     // Check if we need to insert a new keyframe
//     if (NeedNewKeyFrame()) {
//       // std::cout << "--Insert a new key frame! " << std::endl;
//       CreateNewKeyFrame();
//     }
// 
//     // outliers should not be ignored.
//     for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
//       if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
//         mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
//     }
// 
//     if (!mCurrentFrame.mpReferenceKF)
//       mCurrentFrame.mpReferenceKF = mpReferenceKF;
// 
//     mLastFrame = Frame(mCurrentFrame);
//   }
// 
//   // Store frame pose information to retrieve the complete camera trajectory
//   // afterwards.
//   if (!mCurrentFrame.GetPose().Tcw().empty()) {
//     cv::Mat Tcr = mCurrentFrame.GetPose().Tcw() *
//                   mCurrentFrame.mpReferenceKF->GetPose().Twc();
//     mlRelativeFramePoses.push_back(Tcr);
//     mlpReferences.push_back(mpReferenceKF);
//     mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
//     mlbLost.push_back(mState == LOST);
//   } else {
//     mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
//     mlpReferences.push_back(mlpReferences.back());
//     mlFrameTimes.push_back(mlFrameTimes.back());
//     mlbLost.push_back(mState == LOST);
//   }
//   return true;
// }

bool Tracking::Track(const double &timestamp)
{
    // track包含两部分：估计运动、跟踪局部地图
    // mState为tracking的状态，包括 SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    // mLastProcessedState 存储了Tracking最新的状态，用于FrameDrawer中的绘制
    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    // 地图更新时加锁。保证地图不会发生变化
    // 疑问:这样子会不会影响地图的实时更新?
    // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // Step 1：初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            //双目RGBD相机的初始化共用一个函数
            StereoInitialization();
        else
            //单目初始化
            MonocularInitialization();

        //更新帧绘制器中存储的最新状态
        mpFrameDrawer->Update(this);

        //这个状态量在上面的初始化函数中被更新
        if(mState!=OK)
            return false;
    }
    else
    {
        // System is initialized. Track Frame.
        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // mbOnlyTracking等于false表示正常SLAM模式（定位+地图更新），mbOnlyTracking等于true表示仅定位模式
        // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            // Step 2：跟踪进入正常SLAM模式，有地图更新
            // 正常初始化成功
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                // Step 2.1 检查并更新上一帧被替换的MapPoints
                // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrame();

                // Step 2.2 运动模型是空的或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                // 第一个条件,如果运动模型为空,说明是刚初始化开始，或者已经跟丢了
                // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们将重定位帧来恢复位姿
                // mnLastRelocFrameId 上一次重定位的那一帧
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    // 用最近的关键帧来跟踪当前的普通帧
                    // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    // 用最近的普通帧来跟踪当前的普通帧
                    // 根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel(timestamp);
                    if(!bOK)
                        //根据恒速模型失败了，只能根据参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                // 如果跟踪状态不成功,那么就只能重定位了
                // BOW搜索，EPnP求解位姿
                bOK = Relocalization();
            }
        }
        else        
        {
            // Localization Mode: Local Mapping is deactivated
            // Step 2：只进行跟踪tracking，局部地图不工作
            if(mState==LOST)
            {
                // Step 2.1 如果跟丢了，只能重定位
                bOK = Relocalization();
            }
            else    
            {
                // mbVO是mbOnlyTracking为true时的才有的一个变量
                // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                if(!mbVO)
                {
                    // Step 2.2 如果跟踪正常，使用恒速模型 或 参考关键帧跟踪
                    // In last frame we tracked enough MapPoints in the map
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel(timestamp);
                        // ? 为了和前面模式统一，这个地方是不是应该加上
                        // if(!bOK)
                        //    bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        // 如果恒速模型不被满足,那么就只能够通过参考关键帧来定位
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.
                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跪的节奏，既做跟踪又做重定位

                    //MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;
                    //通过重定位方法来跟踪的结果
                    bool bOKReloc = false;
                    
                    //运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;
                    //在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;
                    //运动模型得到的位姿
                    cv::Mat TcwMM;

                    // Step 2.3 当运动模型有效的时候,根据运动模型计算位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel(timestamp);

                        // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.GetPose().Tcw().clone();
                    }

                    // Step 2.4 使用重定位的方法来得到当前帧的位姿
                    bOKReloc = Relocalization();

                    // Step 2.5 根据前面的恒速模型、重定位结果来更新状态
                    if(bOKMM && !bOKReloc)
                    {
                        // 恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        //? 疑似bug！这段代码是不是重复增加了观测次数？后面 TrackLocalMap 函数中会有这些操作
                        // 如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                        if(mbVO)
                        {
                            // 更新当前帧的地图点被观测次数
                            for(int i =0; i<mCurrentFrame.mNumOfKPts; i++)
                            {
                                //如果这个特征点形成了地图点,并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    //增加能观测到该地图点的帧数
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                        mbVO = false;
                    }
                    //有一个成功我们就认为执行成功了
                    bOK = bOKReloc || bOKMM;
                }
            }
        }
        
        // 将最新的关键帧作为当前帧的参考关键帧
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // Step 3：在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.

            // 重定位成功
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        //根据上面的操作来判断是否追踪成功
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Step 4：更新显示线程中的图像、特征点、地图点等信息
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        //只有在成功追踪时才考虑生成关键帧的问题
        if(bOK)
        {
            // Update motion model
            // Step 5：跟踪成功，更新恒速运动模型
            if(!mLastFrame.GetPose().Tcw().empty())
            {
                // 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetPose().Rwc().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetPose().Ow().copyTo(LastTwc.rowRange(0,3).col(3));
                // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                mVelocity = mCurrentFrame.GetPose().Tcw()*LastTwc; 
            }
            else
                //否则速度为空
                mVelocity = cv::Mat();

            //更新显示中的位姿
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose().Tcw());

            // Clean VO matches
            // Step 6：清除观测不到的地图点   
            for(int i=0; i<mCurrentFrame.mNumOfKPts; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // Step 7：清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
            // 步骤6中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }

            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
            // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            // Step 8：检测并插入关键帧，对于双目或RGB-D会产生新的地图点
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
            // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

            //  Step 9 删除那些在bundle adjustment中检测为outlier的地图点
            for(int i=0; i<mCurrentFrame.mNumOfKPts;i++)
            {
                // 这里第一个条件还要执行判断是因为, 前面的操作中可能删除了其中的地图点
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // Step 10 如果初始化后不久就跟踪失败，并且relocation也没有搞定，只能重新Reset
        if(mState==LOST)
        {
            //如果地图中的关键帧信息过少的话,直接重新进行初始化了
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return false;
            }
        }

        //确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }

    
    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    // Step 11：记录位姿信息，用于最后保存所有的轨迹
    if(!mCurrentFrame.GetPose().Tcw().empty())
    {
        // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
        cv::Mat Tcr = mCurrentFrame.GetPose().Tcw()*mCurrentFrame.mpReferenceKF->GetPose().Twc();
        
        //保存各种状态
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        // 如果跟踪失败，则相对位姿使用上一次值
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }
    return true;

}// Tracking 

void Tracking::StereoInitialization() {
  if (mCurrentFrame.mNumOfKPts < 500)
    return;
  // Set Frame pose to the origin
  mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

  // Create KeyFrame
  KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  // Insert KeyFrame in the map
  mpMap->AddKeyFrame(pKFini);

  // Create MapPoints and asscoiate to KeyFrame
  for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
    if (mCurrentFrame.mvDepth[i] <= 0)
      continue;

    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
    pNewMP->AddObservation(pKFini, i);
    pKFini->AddMapPoint(pNewMP, i);
    pNewMP->ComputeDistinctiveDescriptors();
    pNewMP->UpdateNormalAndDepth();
    mpMap->AddMapPoint(pNewMP);

    mCurrentFrame.mvpMapPoints[i] = pNewMP;
  }

  // cout << "New map created with " << mpMap->MapPointsInMap() << " points" <<
  // endl;
  mpLocalMapper->InsertKeyFrame(pKFini);

  mLastFrame = Frame(mCurrentFrame);
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFini;

  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFini;
  mCurrentFrame.mpReferenceKF = pKFini;

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose().Twc());

  mState = OK;
}

void Tracking::DrawPairMatches(const Frame &F1, const Frame &F2,
                               const std::vector<int> &vMatches12) {
  // Reference Frame: 1, Current Frame: 2
  std::vector<cv::KeyPoint> vecKeys1 = F1.mvKeysUn;
  std::vector<cv::KeyPoint> vecKeys2 = F2.mvKeysUn;

  std::vector<std::pair<int, int>> vecIndice12;
  vecIndice12.reserve(vecKeys2.size());
  for (size_t i = 0; i < vMatches12.size(); i++) {
    if (vMatches12[i] >= 0)
      vecIndice12.push_back(make_pair(i, vMatches12[i]));
  }

  cv::Mat imDraw = mImGray.clone();
  if (imDraw.channels() != 3) {
    cv::cvtColor(imDraw, imDraw, cv::COLOR_GRAY2BGR);
  }

  for (size_t i = 0; i < vecIndice12.size(); i++) {
    const cv::KeyPoint &kp1 = vecKeys1[vecIndice12[i].first];
    const cv::KeyPoint &kp2 = vecKeys2[vecIndice12[i].second];
    cv::circle(imDraw, kp1.pt, 3, cv::Scalar(0, 0, 255), 2);
    cv::circle(imDraw, kp2.pt, 3, cv::Scalar(255, 0, 0), 2);
    cv::line(imDraw, kp1.pt, kp2.pt, cv::Scalar(0, 255, 0), 1);
  }
  
  cv::imshow("initial", imDraw);
  cv::waitKey(100);
}

void Tracking::MonocularInitialization() {
  // std::cout << "Monocular Initialization... " << std::endl;
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame.mvKeys.size() < 100)
      return;

    mInitialFrame = Frame(mCurrentFrame);
    mLastFrame = Frame(mCurrentFrame);
    mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
    for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
      mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

    if (mpInitializer)
      delete mpInitializer;

    mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

    std::fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

    return;
  } else {
    if ((int)mCurrentFrame.mvKeys.size() <= 100) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer *>(NULL);
      fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
      return;
    }

    // Find correspondences
    ORBmatcher matcher(0.8, true);
    int nmatches = matcher.SearchForInitialization(
        mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

    // std::cout << "Number of matches: " << nmatches << std::endl;
    // DrawPairMatches(mInitialFrame, mCurrentFrame, mvIniMatches);

    // Check if there are enough correspondences
    if (nmatches < 50) {
      delete mpInitializer;
      mpInitializer = static_cast<Initializer *>(NULL);
      return;
    }
    
    cv::Mat TcwInit, TcwCur;     // Initial Frame Pose and Current Camera pose
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    if (!mpInitializer->Initialize(mCurrentFrame, mvIniMatches, TcwInit, TcwCur,
                                   mvIniP3D, vbTriangulated)) {
      std::cout << "Failed to estimate initial map points!!" << std::endl;
      return;
    }

    for (size_t i = 0; i < mvIniMatches.size(); i++) {
      if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
        mvIniMatches[i] = -1;
        nmatches--;
      }
    }

    std::cout << "Number of initial map points: " << nmatches << std::endl;

    mInitialFrame.SetPose(TcwInit);
    mCurrentFrame.SetPose(TcwCur);

    std::cout << "Ref FrameID 1: " << mInitialFrame.mnId << std::endl;
    std::cout << "Ref FrameID 2: " << mCurrentFrame.mnId << std::endl;

    CreateInitialMapMonocular();
    
//     cv::Mat Rcw; // Current Camera Rotation
//     cv::Mat tcw; // Current Camera Translation
//     vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
// 
//     // Step 5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
//     if(mpInitializer->Initialize(
//         mCurrentFrame,      //当前帧
//         mvIniMatches,       //当前帧和参考帧的特征点的匹配关系
//         Rcw, tcw,           //初始化得到的相机的位姿
//         mvIniP3D,           //进行三角化得到的空间点集合
//         vbTriangulated))    //以及对应于mvIniMatches来讲,其中哪些点被三角化了
//     {
//         // Step 6 初始化成功后，删除那些无法进行三角化的匹配点
//         for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
//         {
//             if(mvIniMatches[i]>=0 && !vbTriangulated[i])
//             {
//                 mvIniMatches[i]=-1;
//                 nmatches--;
//             }
//         }
// 
//         std::cout << "Number of initial map points: " << nmatches << std::endl;
//         
//         // Set Frame Poses
//         // Step 7 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
//         mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
//         // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到相机坐标系的变换矩阵
//         cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
//         Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
//         tcw.copyTo(Tcw.rowRange(0,3).col(3));
//         mCurrentFrame.SetPose(Tcw);
//         
//         std::cout << "Ref FrameID 1: " << mInitialFrame.mnId << std::endl;
//         std::cout << "Ref FrameID 2: " << mCurrentFrame.mnId << std::endl;
// 
//         CreateInitialMapMonocular();
    }
}

void Tracking::CreateInitialMapMonocular() {
  KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
  KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  pKFini->ComputeBoW();
  pKFcur->ComputeBoW();

  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0)
      continue;

    cv::Mat worldPos(mvIniP3D[i]);

    MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, i);
    pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

    pMP->AddObservation(pKFini, i);
    pMP->AddObservation(pKFcur, mvIniMatches[i]);

    pMP->ComputeDistinctiveDescriptors();
    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
    mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

    // Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  std::cout << "New Map created with " << mpMap->MapPointsInMap() << " points"
            << endl;

  Optimizer::GlobalBundleAdjustemnt(mpMap, mInitialFrame.mnId);

  std::cout << "Finish global bundle adjustment !!" << std::endl;

  if (!mCurrentFrame.GetAbsXY().flag && false) {
    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;
    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 20) {
      cout << "Wrong initialization, reseting..." << endl;
      Reset();
      return;
    }
    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose().Tcw();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
      if (vpAllMapPoints[iMP]) {
        MapPoint *pMP = vpAllMapPoints[iMP];
        pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame.SetPose(pKFcur->GetPose().Tcw());
  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mvpLocalMapPoints = mpMap->GetAllMapPoints();
  mpReferenceKF = pKFcur;
  mCurrentFrame.mpReferenceKF = pKFcur;

  mLastFrame = Frame(mCurrentFrame);

  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose().Twc());

  mState = OK;

  // std::cout << "Finish initialization!!" << std::endl;
}

void Tracking::CheckReplacedInLastFrame() {
  for (int i = 0; i < mLastFrame.mNumOfKPts; i++) {
    MapPoint *pMP = mLastFrame.mvpMapPoints[i];

    if (!pMP)
      continue;

    MapPoint *pRep = pMP->GetReplaced();
    if (pRep)
      mLastFrame.mvpMapPoints[i] = pRep;
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // Compute Bag of Words vector
  mCurrentFrame.ComputeBoW();

  // We perform first an ORB matching with the reference keyframe
  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.8, true);
  vector<MapPoint *> vpMapPointMatches;

  int nmatches =
      matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

  if (nmatches < 15)
    return false;

  mCurrentFrame.mvpMapPoints = vpMapPointMatches;

  mCurrentFrame.SetPose(mLastFrame.GetPose().Tcw());

  // std::cout << "--TrackReferenceKeyFrame--" << std::endl;

  Optimizer::PoseOptimization(&mCurrentFrame);

  // std::cout << "Finish Pose optimization!!" << std::endl;

  // Discard outliers
  int nmatchesMap = 0;
  for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
    if (mCurrentFrame.mvpMapPoints[i]) {
      if (mCurrentFrame.mvbOutlier[i]) {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        mCurrentFrame.mvbOutlier[i] = false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
        nmatchesMap++;
    }
  }

  return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
  KeyFrame *pRef = mLastFrame.mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame.SetPose(Tlr * pRef->GetPose().Tcw());

  if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR ||
      !mbOnlyTracking)
    return;

  // Create "visual odometry" MapPoints
  vector<pair<float, int>> vDepthIdx;
  vDepthIdx.reserve(mLastFrame.mNumOfKPts);
  for (int i = 0; i < mLastFrame.mNumOfKPts; i++) {
    const float &z = mLastFrame.mvDepth[i];
    if (z > 0) {
      vDepthIdx.push_back(make_pair(z, i));
    }
  }

  if (vDepthIdx.empty())
    return;

  sort(vDepthIdx.begin(), vDepthIdx.end());

  // If less than 100 close points, we insert the 100 closest ones.
  int nPoints = 0;
  for (size_t j = 0; j < vDepthIdx.size(); j++) {
    int i = vDepthIdx[j].second;

    bool bCreateNew = false;

    MapPoint *pMP = mLastFrame.mvpMapPoints[i];
    if (!pMP)
      bCreateNew = true;
    else if (pMP->Observations() < 1) {
      bCreateNew = true;
    }

    if (bCreateNew) {
      cv::Mat x3D = mLastFrame.UnprojectStereo(i);
      MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

      mLastFrame.mvpMapPoints[i] = pNewMP;

      mlpTemporalPoints.push_back(pNewMP);
      nPoints++;
    } else {
      nPoints++;
    }

    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
      break;
  }
}

// bool Tracking::TrackWithMotionModel(const double &timestamp) {
//   ORBmatcher matcher(0.9, true);
//   UpdateLastFrame();
// 
//   camera::Pose posePredict = m_posePredictor.predict();
//   cv::Mat TcwPredict = posePredict.Tcw();
// 
//   mCurrentFrame.SetPose(TcwPredict);
// 
//   fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(),
//        static_cast<MapPoint *>(NULL));
// 
//   // Project points seen in previous frame
//   int th = 7;
//   if (mSensor != System::STEREO)
//     th = 15;
//   int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th,
//                                             mSensor == System::MONOCULAR);
// 
//   // If few matches, uses a wider window search
//   if (nmatches < Param::MIN_PAIRWISE_MATCHES_NUMBER) {
//     std::fill(mCurrentFrame.mvpMapPoints.begin(),
//               mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
//     nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th,
//                                           mSensor == System::MONOCULAR);
//   }
//   if (nmatches < Param::MIN_PAIRWISE_MATCHES_NUMBER)
//     return false;
// 
//   // Optimize frame pose with all matches
//   // std::cout << "--TrackWithMotionModel--PoseOptimization" << std::endl;
//   Optimizer::PoseOptimization(&mCurrentFrame);
// 
//   // Discard outliers
//   int nmatchesMap = 0;
//   for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
//     if (!mCurrentFrame.mvpMapPoints[i])
//       continue;
// 
//     if (mCurrentFrame.mvbOutlier[i]) {
//       MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
// 
//       mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
//       mCurrentFrame.mvbOutlier[i] = false;
//       pMP->mbTrackInView = false;
//       pMP->mnLastFrameSeen = mCurrentFrame.mnId;
//       nmatches--;
//     } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
//       nmatchesMap++;
//   }
//   return nmatchesMap >= 10;
// }
bool Tracking::TrackWithMotionModel(const double &timestamp)
{
    // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points
    // Step 1：更新上一帧的位姿；对于双目或rgbd摄像头，还会根据深度值生成临时地图点
    UpdateLastFrame();

    // 根据恒速模型估计当前帧初始位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.GetPose().Tcw());
    
    //清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    // 设置匹配过程中的搜索半径
    int th;
    if(mSensor!=System::STEREO)
        th=15;//单目
    else
        th=7;//双目

    // Step 2：根据上一帧特征点对应地图点进行投影匹配
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    // 如果匹配点太少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR); // 2*th
    }

    // 如果就算是这样还是不能够获得足够的匹配点,那么就认为跟踪失败
    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    // Step 3：优化当前帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // Step 4：优化位姿后剔除地图点中外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.mNumOfKPts; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                //累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        // 如果在纯定位过程中追踪的地图点非常少,那么这里的 mbVO 标志就会置位
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    // Step 5：匹配超过10个点就认为成功
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap() // retrieve the local map and try to find
                               // matches to points in the local map.
{
  UpdateLocalMap();
  SearchLocalPoints();

  // std::cout << "--TrackLocalMap--PoseOptimization" << std::endl;
  Optimizer::PoseOptimization(&mCurrentFrame);

  mnMatchesInliers = 0;
  for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
    if (!mCurrentFrame.mvpMapPoints[i])
      continue;

    if (!mCurrentFrame.mvbOutlier[i]) {
      mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
      if (!mbOnlyTracking) {
        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
          mnMatchesInliers++;
      } else
        mnMatchesInliers++;
    } else if (mSensor == System::STEREO)
      mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
  }

  if (mnMatchesInliers < Param::MIN_2D3D_MATCHES_NUMBER) {
    std::cout << "--TrackLocalMap: mnMatchesInliers: " << mnMatchesInliers
              << std::endl;
    return false;
  }
  return true;
}


bool Tracking::NeedNewKeyFrame() {
  if (mbOnlyTracking)
    return false;

  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

  const int nKFs = mpMap->KeyFramesInMap();

  // Do not insert keyframes if not enough frames have passed from last
  // relocalisation
  if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames &&
      nKFs > mMaxFrames) {
    return false;
  }
  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2)
    nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Check how many "close" points are being tracked and how many could be
  // potentially created.
  int nNonTrackedClose = 0;
  int nTrackedClose = 0;
  if (mSensor != System::MONOCULAR) {
    for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
      if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
        if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
          nTrackedClose++;
        else
          nNonTrackedClose++;
      }
    }
  }

  bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

  // Thresholds
  float thRefRatio = 0.75f;
  if (nKFs < 2)
    thRefRatio = 0.4f;

  if (mSensor == System::MONOCULAR)
    thRefRatio = 0.9f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe
  // insertion
  const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames &&
                    bLocalMappingIdle);
  // Condition 1c: tracking is weak
  const bool c1c =
      mSensor != System::MONOCULAR &&
      (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
  // Condition 2: Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 =
      ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) &&
       mnMatchesInliers > 15);

  // float angle = angelOfTwoVector(mCurrentFrame.GetPose().Ow(),
  // mpLastKeyFrame->GetPose().Ow()); std::cout << "angle: " << angle <<
  // std::endl;

  if ((c1a || c1b || c1c) && c2) {
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      if (mSensor != System::MONOCULAR) {
        if (mpLocalMapper->KeyframesInQueue() < 3)
          return true;
        else
          return false;
      } else
        return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  if (!mpLocalMapper->SetNotStop(true))
    return;

  KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame.mpReferenceKF = pKF;

  if (mSensor != System::MONOCULAR) {
    // We sort points by the measured depth by the stereo/RGBD sensor.
    // We create all those MapPoints whose depth < mThDepth.
    // If there are less than 100 close points we create the 100 closest.
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mCurrentFrame.mNumOfKPts);
    for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
      const float &z = mCurrentFrame.mvDepth[i];
      if (z <= 0)
        continue;

      vDepthIdx.push_back(make_pair(z, i));
    }

    if (!vDepthIdx.empty()) {
      sort(vDepthIdx.begin(), vDepthIdx.end());
      int nPoints = 0;
      for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
          bCreateNew = true;
        else if (pMP->Observations() < 1) {
          bCreateNew = true;
          mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
        }

        if (bCreateNew) {
          cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
          MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
          pNewMP->AddObservation(pKF, i);
          pKF->AddMapPoint(pNewMP, i);
          pNewMP->ComputeDistinctiveDescriptors();
          pNewMP->UpdateNormalAndDepth();
          mpMap->AddMapPoint(pNewMP);

          mCurrentFrame.mvpMapPoints[i] = pNewMP;
          nPoints++;
        } else {
          nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
          break;
      }
    }
  }

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame.mnId;
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
                                    vend = mCurrentFrame.mvpMapPoints.end();
       vit != vend; vit++) {
    MapPoint *pMP = *vit;
    if (!pMP)
      continue;

    if (pMP->isBad()) {
      *vit = static_cast<MapPoint *>(NULL);
    } else {
      pMP->IncreaseVisible();
      pMP->mnLastFrameSeen = mCurrentFrame.mnId;
      pMP->mbTrackInView = false;
    }
  }

  int nToMatch = 0;

  // Project points in frame and check its visibility
  for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(),
                                    vend = mvpLocalMapPoints.end();
       vit != vend; vit++) {
    MapPoint *pMP = *vit;
    if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
      continue;
    if (pMP->isBad())
      continue;
    // Project (this fills MapPoint variables for matching)
    if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
      nToMatch++;
    }
  }

  if (nToMatch > 0) {
    ORBmatcher matcher(0.8);
    float th = 1.0;
    if (mSensor == System::RGBD)
      th = 3;
    // If the camera has been relocalised recently, perform a coarser search
    if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
      th = 5;
    matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
  }
}

void Tracking::UpdateLocalMap() {
  // This is for visualization
  mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

  // Update
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
  mvpLocalMapPoints.clear();

  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                          itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    KeyFrame *pKF = *itKF;
    const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

    for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(),
                                            itEndMP = vpMPs.end();
         itMP != itEndMP; itMP++) {
      MapPoint *pMP = *itMP;
      if (!pMP)
        continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
        continue;
      if (!pMP->isBad()) {
        mvpLocalMapPoints.push_back(pMP);
        pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  map<KeyFrame *, int> keyframeCounter;
  for (int i = 0; i < mCurrentFrame.mNumOfKPts; i++) {
    if (!mCurrentFrame.mvpMapPoints[i])
      continue;

    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
    if (!pMP->isBad()) {
      const map<KeyFrame *, size_t> observations = pMP->GetObservations();
      for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(),
                                                   itend = observations.end();
           it != itend; it++)
        keyframeCounter[it->first]++;
    } else {
      mCurrentFrame.mvpMapPoints[i] = NULL;
    }
  }

  if (keyframeCounter.empty())
    return;

  int max = 0;
  KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                            itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFrame *pKF = it->first;

    if (pKF->isBad())
      continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
  }

  // Include also some not-already-included keyframes that are neighbors to
  // already-included keyframes
  for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                          itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 50)
      break;

    KeyFrame *pKF = *itKF;

    const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(),
                                            itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame *pNeighKF = *itNeighKF;
      if (pNeighKF->isBad())
        continue;

      if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pNeighKF);
        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }

    const set<KeyFrame *> spChilds = pKF->GetChilds();
    for (set<KeyFrame *>::const_iterator sit = spChilds.begin(),
                                         send = spChilds.end();
         sit != send; sit++) {
      KeyFrame *pChildKF = *sit;
      if (pChildKF->isBad())
        continue;

      if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pChildKF);
        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }

    KeyFrame *pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        break;
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame.mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  mCurrentFrame.ComputeBoW();

  // Relocalization is performed when tracking is lost
  vector<KeyFrame *> vpCandidateKFs =
      mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

  if (vpCandidateKFs.empty())
    return false;

  const int nKFs = vpCandidateKFs.size();

  // If enough matches are found we setup a PnP solver
  ORBmatcher matcher(0.75, true);

  vector<PnPsolver *> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<vector<MapPoint *>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame *pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      int nmatches =
          matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        PnPsolver *pSolver =
            new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
        vpPnPsolvers[i] = pSolver;
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;
  ORBmatcher matcher2(0.9, true);

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i])
        continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      PnPsolver *pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (Tcw.empty())
        continue;

      mCurrentFrame.SetPose(Tcw);

      set<MapPoint *> sFound;
      const int np = vbInliers.size();
      for (int j = 0; j < np; j++) {
        if (vbInliers[j]) {
          mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
          sFound.insert(vvpMapPointMatches[i][j]);
        } else
          mCurrentFrame.mvpMapPoints[j] = NULL;
      }

      int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
      if (nGood < 10)
        continue;

      for (int io = 0; io < mCurrentFrame.mNumOfKPts; io++)
        if (mCurrentFrame.mvbOutlier[io])
          mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

      // If few inliers, search by projection in a coarse window and optimize
      // again
      if (nGood < 50) {
        int nadditional = matcher2.SearchByProjection(
            mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

        if (nadditional + nGood >= 50) {
          nGood = Optimizer::PoseOptimization(&mCurrentFrame);

          // If many inliers but still not enough, search by projection again in
          // a narrower window the camera has been already optimized with many
          // points
          if (nGood > 30 && nGood < 50) {
            sFound.clear();
            for (int ip = 0; ip < mCurrentFrame.mNumOfKPts; ip++)
              if (mCurrentFrame.mvpMapPoints[ip])
                sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
            nadditional = matcher2.SearchByProjection(
                mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

            // Final optimization
            if (nGood + nadditional >= 50) {
              nGood = Optimizer::PoseOptimization(&mCurrentFrame);

              for (int io = 0; io < mCurrentFrame.mNumOfKPts; io++)
                if (mCurrentFrame.mvbOutlier[io])
                  mCurrentFrame.mvpMapPoints[io] = NULL;
            }
          }
        }
      }

      // If the pose is supported by enough inliers stop ransacs and continue
      if (nGood >= 50) {
        bMatch = true;
        break;
      }
    }
  }

  if (!bMatch)
    return false;
  else {
    mnLastRelocFrameId = mCurrentFrame.mnId;
    return true;
  }
}

void Tracking::Reset() {
  cout << "\nSystem Reseting..." << endl;

  if (mpViewer)
    mpViewer->Release();
  cout << "Done with viewer release!" << std::endl;

  // Reset Local Mapping
  mpLocalMapper->RequestReset();
  while (!mpLocalMapper->isReseted())
    usleep(50);
  cout << "Done with Local Mapper reset!" << std::endl;

  // Clear BoW Database
  mpKeyFrameDB->clear();
  cout << "Done with Database reset!" << std::endl;

  mpMap->clear();
  cout << "Done with map points clear!" << std::endl;

  KeyFrame::nNextId = 0;
  Frame::nNextId = 0;
  mState = NO_IMAGES_YET;

  if (mpInitializer) {
    delete mpInitializer;
    mpInitializer = static_cast<Initializer *>(NULL);
  }

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
  cout << "Done with reset!" << std::endl;
}

void Tracking::ChangeCalibration(const string &strSettingPath) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  cv::Mat DistCoef(4, 1, CV_32F);
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    DistCoef.resize(5);
    DistCoef.at<float>(4) = k3;
  }
  DistCoef.copyTo(mDistCoef);

  mbf = fSettings["Camera.bf"];

  Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) { mbOnlyTracking = flag; }

void Tracking::Normal_estimate(){
    // 计算第一组质心
    Vec3 p_slam_1, p_ref_1;
    for(int i=0; i<10;i++){
        p_slam_1 = p_slam_1 + nprameter.pose_slam[i];
        p_ref_1  = p_ref_1  + nprameter.pose_ref[i];
    }
    
    // 计算第二组质心
    Vec3 p_slam_2= p_slam_1, p_ref_2=  p_ref_1;
    for(int i=10; i<20;i++){
        p_slam_2 = p_slam_2 + nprameter.pose_slam[i];
        p_ref_2  = p_ref_2  + nprameter.pose_ref[i];
    }

    // 计算第三组质心
    Vec3 p_slam_3= p_slam_2, p_ref_3=  p_ref_2;
    for(int i=20; i<30;i++){
        p_slam_3 = p_slam_3 + nprameter.pose_slam[i];
        p_ref_3  = p_ref_3  + nprameter.pose_ref[i];
    }
    
    // 质心归一化处理
    p_ref_1 = (p_ref_1/10 - nprameter.Init_Traslation);
    p_ref_2 = (p_ref_2/20 - nprameter.Init_Traslation);
    p_ref_3 = (p_ref_3/30 - nprameter.Init_Traslation);
    
    p_slam_1 = p_slam_1 / 10;
    p_slam_2 = p_slam_2 / 20;
    p_slam_3 = p_slam_3 / 30;

    // 计算待奇异分解的矩阵
    Eigen::Matrix3d W;
    W= p_ref_1*p_slam_1.transpose() + p_ref_2*p_slam_2.transpose() +p_ref_3*p_slam_3.transpose();
    
    // 奇异性分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV );
	Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
    
    // 归一化参数输出
    nprameter.Init_Rotation = U*V.transpose();
    cout << " 你哈哈或或或或或或或！！！！！！！！"<<endl;
    
} 


} // namespace ORB_SLAM2
