#include "loadArg.h"

#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <vector>

#include "3rdParty/stlplus3/filesystemSimplified/file_system.hpp"

bool loadArg::loadSLAMSetting(const std::string &sSetting, cv::Mat &matK,
                              cv::Mat &matDistCoef, SLAMConfig &slamParam,
                              bool bDebug) {
  if (!stlplus::file_exists(sSetting)) {
    std::cout << "SLAM setting file is not exist!!" << std::endl;
    return false;
  }
  std::cout << "The setting file path: " << sSetting << std::endl << std::endl;

  // Load camera parameters from settings file
  cv::FileStorage fSettings(sSetting, cv::FileStorage::READ);
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];
  matK = cv::Mat::eye(3, 3, CV_32F);
  matK.at<float>(0, 0) = fx;
  matK.at<float>(1, 1) = fy;
  matK.at<float>(0, 2) = cx;
  matK.at<float>(1, 2) = cy;

  matDistCoef = cv::Mat::zeros(4, 1, CV_32F);
  matDistCoef.at<float>(0) = fSettings["Camera.k1"];
  matDistCoef.at<float>(1) = fSettings["Camera.k2"];
  matDistCoef.at<float>(2) = fSettings["Camera.p1"];
  matDistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if (k3 != 0) {
    matDistCoef.resize(5);
    matDistCoef.at<float>(4) = k3;
  }

  // load slam setting pamaters
  float fps = fSettings["Camera.fps"];
  SettingParams setParam;
  setParam.fps = fps == 0 ? 10 : fps;
  setParam.bf = fSettings["Camera.bf"];  // for stereo in case
  int nRGB = fSettings["Camera.RGB"];
  setParam.bRGB = nRGB;
  float fDepthMapFactor = fSettings["DepthMapFactor"];
  setParam.depthMapFactor =
      fabs(fDepthMapFactor) < 1e-5 ? 1.0 : 1.0f / fDepthMapFactor;
  setParam.thDepth = setParam.bf * (float)fSettings["ThDepth"] / fx;

  // load ORB detector parameters
  ORBParams orbParam;
  orbParam.numFeatures = fSettings["ORBextractor.nFeatures"];
  orbParam.scaleFactor = fSettings["ORBextractor.scaleFactor"];
  orbParam.numLevels = fSettings["ORBextractor.nLevels"];
  orbParam.numIniThFAST = fSettings["ORBextractor.iniThFAST"];
  orbParam.numMinThFAST = fSettings["ORBextractor.minThFAST"];

  // load Tci
  slamParam.settingParam = setParam;
  slamParam.orbFeatParam = orbParam;

  // for test
  if (bDebug) {
    std::cout << std::endl << "Camera Parameters: " << std::endl;
    std::cout << "- fx: " << fx << std::endl;
    std::cout << "- fy: " << fy << std::endl;
    std::cout << "- cx: " << cx << std::endl;
    std::cout << "- cy: " << cy << std::endl;
    std::cout << "- k1: " << matDistCoef.at<float>(0) << std::endl;
    std::cout << "- k2: " << matDistCoef.at<float>(1) << std::endl;
    if (matDistCoef.rows == 5)
      std::cout << "- k3: " << matDistCoef.at<float>(4) << std::endl;
    std::cout << "- p1: " << matDistCoef.at<float>(2) << std::endl;
    std::cout << "- p2: " << matDistCoef.at<float>(3) << std::endl;
    std::cout << "- fps: " << fps << std::endl;

    std::cout << std::endl << "ORB Extractor Parameters: " << std::endl;
    std::cout << "- Number of Features: " << orbParam.numFeatures << std::endl;
    std::cout << "- Scale Levels: " << orbParam.numLevels << std::endl;
    std::cout << "- Scale Factor: " << orbParam.scaleFactor << std::endl;
    std::cout << "- Initial Fast Threshold: " << orbParam.numIniThFAST
              << std::endl;
    std::cout << "- Minimum Fast Threshold: " << orbParam.numMinThFAST
              << std::endl;
  }
  return true;
}

bool loadArg::loadImageTime(const std::string &sTimeList,
                            const std::vector<ViewID> &viewIDs,
                            std::map<ViewID, double> &mapTime) {
  mapTime.clear();
  std::ifstream in(sTimeList.c_str());
  if (!in.is_open() || viewIDs.empty()) {
    std::cerr << std::endl
              << "Impossible to read the specified file." << std::endl;
    return false;
  }

  for (size_t i = 0; i < viewIDs.size(); ++i) {
    double time;
    in >> time;
    mapTime.insert(std::make_pair(viewIDs[i], time));
  }
  in.close();
  return !(mapTime.empty());
}