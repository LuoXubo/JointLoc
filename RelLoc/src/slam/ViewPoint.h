#ifndef VIEWPOINT_H
#define VIEWPOINT_H

#include <opencv2/core/core.hpp>

#include "common/Camera.h"

struct NormalPrameter
{

  std::vector<Vec3> pose_slam;
  std::vector<Vec3> pose_ref;
  int Num_Ready;
  int Num_Need;
  bool NP_Ready;

  int wait;
  bool use;

  // double Init_Scale;
  // Eigen::Vector3d Init_Traslation;
  // Eigen::Matrix3d Init_Rotation;

  Eigen::Vector3d l2g_translation;
  Eigen::Matrix3d l2g_rotation;

  std::vector<Eigen::Vector3d> abs;
  std::vector<Eigen::Vector3d> ref;

  // 全局定位传过来的关键帧信息
  std::vector<int> kfIDs;
  std::vector<Vec3> kfPos;
  std::vector<Vec4> kfOrin;
  std::vector<float> kfConf;
};

namespace camera
{
  // the position and orientation of image in world [R|C] t = -RC
  class Pose
  {
  public:
    Pose()
    {
      m_Tcw = cv::Mat::eye(4, 4, CV_32F);
      m_Rcw = m_Rwc = cv::Mat::eye(3, 3, CV_32F);
      m_tcw = m_Ow = cv::Mat::zeros(3, 1, CV_32F);
    }

    Pose(cv::Mat Tcw)
    {
      m_Tcw = Tcw.clone();
      m_Rcw = m_Tcw.rowRange(0, 3).colRange(0, 3);
      m_Rwc = m_Rcw.t();
      m_tcw = m_Tcw.rowRange(0, 3).col(3);
      m_Ow = -m_Rwc * m_tcw;

      m_Twc = cv::Mat::eye(4, 4, m_Tcw.type());
      m_Rwc.copyTo(m_Twc.rowRange(0, 3).colRange(0, 3));
      m_Ow.copyTo(m_Twc.rowRange(0, 3).col(3));
    }

    const cv::Mat &Tcw() const { return m_Tcw; }
    const cv::Mat &Rcw() const { return m_Rcw; }
    const cv::Mat &Rwc() const { return m_Rwc; }
    const cv::Mat &Ow() const { return m_Ow; } // camera center
    const cv::Mat &tcw() const { return m_tcw; }
    const cv::Mat &Twc() const { return m_Twc; }

    const bool empty() const { return m_Tcw.empty(); }

  private:
    // a 4x4 matrix m_Tcw = [R, t]
    //                      [0, 1]
    cv::Mat m_Tcw;
    cv::Mat m_Twc; // inverse pose
    cv::Mat m_Rcw; // rotation from world to camera
    cv::Mat m_tcw; // translation for camera
    cv::Mat m_Rwc; // rotation from camera to world
    cv::Mat m_Ow;  // camera center in the world

    cv::Mat m_cov; // covariance for r and c | 6x6
  };

  // Absolute XY measurement.
  struct AbsXY
  {
    bool flag = false;

    Vec3 xyz;
    Qua qwxyz;
    Mat3 sqrt_cov;
  };

} // namespace camera

#endif // VIEWPOINT_H
