#include "PosePredictor.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "Converter.h"

using namespace tracking;
using namespace BASE;
PosePredictor::PosePredictor() { m_flag = false; }

void PosePredictor::initial(const Pose& p0, const Pose& p1) {
  m_refPose = p0;
  m_curPose = p1;

  m_velocity = m_curPose.Tcw() * m_refPose.Twc();
  m_flag = true;
}

Pose PosePredictor::predict() {
  cv::Mat Tcw = m_velocity * m_curPose.Tcw();

  return Pose(Tcw);
}

void PosePredictor::update(const Pose& p) {
  if (p.empty()) {
    m_flag = false;
    return;
  }

  m_refPose = m_curPose;
  m_curPose = p;
  m_velocity = m_curPose.Tcw() * m_refPose.Twc();
}

void PosePredictor::correct(const cv::Mat& velocity) { m_velocity = velocity; }

bool PosePredictor::isValid() { return m_flag; }

#include <opencv2/video/video.hpp>
#include <opencv2/videostab/global_motion.hpp>

PosePredictor_KF::PosePredictor_KF() { m_flag = false; }

void PosePredictor_KF::initial(const Pose& p0, const Pose& p1) {
  // Kalman Filter
  m_KF.init(g_stateSize, g_measSize, 0, g_type);

  // Transition Matrix A
  cv::Mat matA = cv::Mat::eye(g_stateSize, g_stateSize, g_type);
  matA.at<float>(0, 7) = 1.0f;
  matA.at<float>(1, 8) = 1.0f;
  matA.at<float>(2, 9) = 1.0f;
  matA.at<float>(3, 10) = 1.0f;
  matA.at<float>(4, 11) = 1.0f;
  matA.at<float>(5, 12) = 1.0f;
  matA.at<float>(6, 13) = 1.0f;

  // std::cout << "!!!A: " << std::endl << matA;

  m_KF.transitionMatrix = matA;

  // Measure Matrix H
  cv::setIdentity(m_KF.measurementMatrix);
  cv::setIdentity(m_KF.processNoiseCov,
                  cv::Scalar(1e-4));  // Process Noise Covariance Matrix Q
  cv::setIdentity(m_KF.measurementNoiseCov,
                  cv::Scalar(1e-1));  // Measures Noise Covariance Matrix R
  cv::setIdentity(m_KF.errorCovPost, cv::Scalar(1.0));
  cv::setIdentity(m_KF.errorCovPre, cv::Scalar(1.0));

  // Initialization
  //    cv::Mat velocity = p1.Tcw()*p0.Twc();
  //    std::cout << "Velocity: \n" << velocity << std::endl;

  cv::Mat dR = p1.Rwc() * p0.Rcw();
  cv::Mat dP = p1.Ow() - p0.Ow();

  Eigen::Quaterniond dQ = Converter::toQuaternion(dR);
  Eigen::Quaterniond sQ = Converter::toQuaternion(p1.Rwc());

  cv::Mat state(g_stateSize, 1, g_type);
  state.at<float>(0, 0) = sQ.w();
  state.at<float>(1, 0) = sQ.x();
  state.at<float>(2, 0) = sQ.y();
  state.at<float>(3, 0) = sQ.z();

  for (int i = 0; i < 3; ++i)
    state.at<float>(i + 4, 0) = p1.Ow().at<float>(i, 0);

  state.at<float>(7, 0) = dQ.w();
  state.at<float>(8, 0) = dQ.x();
  state.at<float>(9, 0) = dQ.y();
  state.at<float>(10, 0) = dQ.z();

  for (int i = 0; i < 3; ++i) state.at<float>(i + 11, 0) = dP.at<float>(i, 0);

  m_KF.statePre = state;

  std::cout << "init_state: " << m_KF.statePre.t() << std::endl;

  m_flag = true;
}

Pose PosePredictor_KF::predict() {
  cv::Mat state(g_stateSize, 1, g_type);
  state = m_KF.predict();

  std::cout << "predit_state: " << state.t() << std::endl;

  Eigen::Quaterniond q;
  q.w() = state.at<float>(0, 0);
  q.x() = state.at<float>(1, 0);
  q.y() = state.at<float>(2, 0);
  q.z() = state.at<float>(3, 0);
  cv::Mat Rwc = Converter::toRotationMatrix(q);
  cv::Mat Rcw = Rwc.t();
  cv::Mat Ow = cv::Mat::zeros(3, 1, g_type);
  for (int i = 4; i < 7; ++i) Ow.at<float>(i - 4, 0) = state.at<float>(i, 0);

  cv::Mat tcw = -Rcw * Ow;

  cv::Mat matT = cv::Mat::eye(4, 4, CV_32F);
  Rcw.copyTo(matT.rowRange(0, 3).colRange(0, 3));
  tcw.copyTo(matT.rowRange(0, 3).col(3));

  std::cout << "predit Rcw :\n " << Rcw << std::endl;

  return Pose(matT);
}

void PosePredictor_KF::update(const Pose& p) {
  if (p.empty()) {
    m_flag = false;
    return;
  }

  cv::Mat meas(g_measSize, 1, g_type);

  Eigen::Quaterniond sQ = Converter::toQuaternion(p.Rwc());
  meas.at<float>(0, 0) = sQ.w();
  meas.at<float>(1, 0) = sQ.x();
  meas.at<float>(2, 0) = sQ.y();
  meas.at<float>(3, 0) = sQ.z();

  for (int i = 0; i < 3; ++i) meas.at<float>(i + 4, 0) = p.Ow().at<float>(i, 0);

  m_KF.correct(meas);
}

bool PosePredictor_KF::isValid() { return m_flag; }
