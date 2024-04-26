#include "Converter.h"

#include <ceres/rotation.h>

#include <opencv2/core/eigen.hpp>

#include "common/Types.h"

namespace BASE {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

Mat4 Converter::toMatrix4d(const cv::Mat &cvT) {
  Mat4 T;
  T << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
      cvT.at<float>(0, 3), cvT.at<float>(1, 0), cvT.at<float>(1, 1),
      cvT.at<float>(1, 2), cvT.at<float>(1, 3), cvT.at<float>(2, 0),
      cvT.at<float>(2, 1), cvT.at<float>(2, 2), cvT.at<float>(2, 3),
      cvT.at<float>(3, 0), cvT.at<float>(3, 1), cvT.at<float>(3, 2),
      cvT.at<float>(3, 3);

  return T;
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m) {
  cv::Mat cvMat(4, 4, CV_32F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 3> &m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++) cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Sim3 &Sim3) {
  Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
  Eigen::Vector3d eigt = Sim3.translation();
  double s = Sim3.scale();
  return toCvSE3(s * eigR, eigt);
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                           const Eigen::Matrix<double, 3, 1> &t) {
  cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cvMat.at<float>(i, j) = R(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    cvMat.at<float>(i, 3) = t(i);
  }

  return cvMat.clone();
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvPoint.x, cvPoint.y, cvPoint.z;

  return v;
}

Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
  Eigen::Matrix<double, 3, 3> M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

Eigen::Quaterniond Converter::toQuaternion(const cv::Mat &M) {
  Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
  return Eigen::Quaterniond(eigMat);
}

cv::Mat Converter::toRotationMatrix(const Eigen::Quaterniond &q) {
  Eigen::Matrix3d matR;
  double a = q.w(), b = q.x(), c = q.y(), d = q.z();
  matR << a * a + b * b - c * c - d * d, 2 * (b * c - a * d),
      2 * (b * d + a * c), 2 * (b * c + a * d), a * a - b * b + c * c - d * d,
      2 * (c * d - a * b), 2 * (b * d - a * c), 2 * (c * d + a * b),
      a * a - b * b - c * c + d * d;

  // Eigen::Matrix<double,3,3> matR = qTmp.toRotationMatrix();

  return toCvMat(matR);
}

void Converter::toSixDoFPose(const cv::Mat &cvT, Vec3 &C, Vec3 &angle) {
  Mat4 matPose;
  cv::cv2eigen(cvT, matPose);

  Mat3 R = matPose.block<3, 3>(0, 0);
  Vec3 t = matPose.block<3, 1>(0, 3);

  C = -R.transpose() * t;

  //    std::cout << "center: " << C.transpose() << std::endl;
  //    std::cout << "rotate:\n " << R << std::endl;
  //    std::cout << "matPose: \n" << matPose << std::endl;

  ceres::RotationMatrixToAngleAxis(R.data(), angle.data());
}

Vec3 Converter::toAngleAxis(const cv::Mat &R) {
  Vec3 angle;
  Mat3 matR;
  cv::cv2eigen(R, matR);
  ceres::RotationMatrixToAngleAxis(matR.data(), angle.data());
  return angle;
}
Vec3 Converter::toAngleAxis(const Mat3 &R) {
  Vec3 angle;
  ceres::RotationMatrixToAngleAxis(R.data(), angle.data());
  return angle;
}
Mat3 Converter::AngleAxistoMatrix(const Vec3 &angle) {
  Mat3 R;
  ceres::AngleAxisToRotationMatrix(angle.data(), R.data());
  return R;
}

Mat4 Converter::toMatrixPose(const double *const p) {
  Mat3 R;
  ceres::AngleAxisToRotationMatrix(p + 3, R.data());
  Vec3 C(p[0], p[1], p[2]);

  Mat4 tmpT = Mat4::Identity();
  tmpT.block<3, 3>(0, 0) = R;
  tmpT.block<3, 1>(0, 3) = -R * C;

  return tmpT;
}

}  // namespace BASE
