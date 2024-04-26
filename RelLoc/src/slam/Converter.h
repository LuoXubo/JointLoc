#ifndef CONVERTER_H
#define CONVERTER_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "sim3.h"

namespace BASE {

class Converter {
 public:
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

  static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 3> &m);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
  static cv::Mat toCvMat(const Sim3 &Sim3);

  static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                         const Eigen::Matrix<double, 3, 1> &t);

  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
  static Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &cvT);

  static Eigen::Quaterniond toQuaternion(const cv::Mat &M);

  static cv::Mat toRotationMatrix(const Eigen::Quaterniond &q);

  static void toSixDoFPose(const cv::Mat &cvT, Eigen::Vector3d &C,
                           Eigen::Vector3d &angle);
  static Eigen::Matrix<double, 4, 4> toMatrixPose(const double *p);

  static Vec3 toAngleAxis(const cv::Mat &R);
  static Vec3 toAngleAxis(const Mat3 &R);
  static Mat3 AngleAxistoMatrix(const Vec3 &angle);
};

}  // namespace BASE

#endif  // CONVERTER_H
