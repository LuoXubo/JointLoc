#ifndef MODEBA_H
#define MODEBA_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"


namespace ORB_SLAM2 {

namespace Optimizer {
bool GlobalBundleAdjustemnt(Map *pMap, FrameID baseFrameId,
                            const unsigned long nLoopKF = 0,
                            const bool bRobust = true);

int PoseOptimization(Frame *pFrame, bool bRobust = true);

bool LocalBundleAdjustment(KeyFrame *pKF, FrameID baseFrameId, bool *pbStopFlag,
                           Map *pMap);

/*
bool OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                            const LoopClosing::KeyFrameAndPose
&NonCorrectedSim3, const LoopClosing::KeyFrameAndPose &CorrectedSim3, const
map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale);

int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *>
&vpMatches1, Sim3 &sim12, const float th2, const bool bFixScale);
                 */
}  // namespace Optimizer
// Frame m_InitialFrame;
}  // namespace ORB_SLAM2

#endif  // MODEBA_H
