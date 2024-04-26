#ifndef LOAD_ARG_H
#define LOAD_ARG_H

#include <opencv2/core/core.hpp>

#include "common/Camera.h"

// load pameters from cfg files
namespace loadArg {

bool loadSLAMSetting(const std::string &sSetting, cv::Mat &matK,
                     cv::Mat &matDistCoef, SLAMConfig &slamParam,
                     bool bDebug = true);

bool loadImageTime(const std::string &sTimeList,
                   const std::vector<ViewID> &viewIDs,
                   std::map<ViewID, double> &mapTime);
}  // namespace loadArg

#endif
