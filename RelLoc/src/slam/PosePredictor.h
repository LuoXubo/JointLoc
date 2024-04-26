#ifndef POSEPREDICTOR_H
#define POSEPREDICTOR_H

#include <opencv2/video/tracking.hpp>

#include "ViewPoint.h"

namespace tracking {
using namespace camera;

class PosePredictor {
 public:
  PosePredictor();

  void initial(const Pose& p0, const Pose& p1);
  Pose predict();
  void update(const Pose& p);
  void correct(const cv::Mat& velocity);

  bool isValid();

 private:
  bool solve_v(const std::vector<double>& vecTime,
               const std::vector<double>& vecObsev, double& v);

 private:
  // velocity
  cv::Mat m_velocity;

  Pose m_refPose;
  Pose m_curPose;

  double m_refTime;
  double m_curTime;

  bool m_flag;
};

// KF based predictor.
class PosePredictor_KF {
 public:
  PosePredictor_KF();

  void initial(const Pose& p0, const Pose& p1);
  Pose predict();
  void update(const Pose& p);
  bool isValid();

 private:
  const size_t g_stateSize =
      14;  // [quaternaion(4x1), x,y,z, dw, di, dj, dk, v_x, v_y, v_z,]
  const size_t g_measSize = 7;
  const size_t g_type = CV_32F;

  cv::KalmanFilter m_KF;
  bool m_flag;
};

}  // namespace tracking

#endif  // POSEPREDICTOR_H
