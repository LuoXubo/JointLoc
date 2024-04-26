#ifndef CAMERA_H
#define CAMERA_H

#include <map>
#include <memory>

#include "Types.h"

namespace camera {

// camera intrinsic parameters
class CameraIntrinsic {
 public:
  CameraIntrinsic() { m_K = Mat3::Zero(); }
  CameraIntrinsic(const Mat3& K, int cols, int rows)
      : m_K(K), m_cols(cols), m_rows(rows) {}
  CameraIntrinsic(const Mat3& K, const Mat& dist, int cols, int rows)
      : m_K(K), m_distort(dist), m_cols(cols), m_rows(rows) {}

  ~CameraIntrinsic() {}

  const Mat3& getK() const { return m_K; }

  const std::pair<size_t, size_t> imSize() const {
    std::pair<size_t, size_t> imgSize;
    imgSize.first = m_cols;
    imgSize.second = m_rows;
    return imgSize;
  }
  const int getImWidth() const { return m_cols; }

  const int getImHeight() const { return m_rows; }

  const Mat& getDistortParam() const { return m_distort; }

  const float getFocal() const { return m_K(0, 0); }

  const float getCx() const { return m_K(0, 2); }

  const float getCy() const { return m_K(1, 2); }

 private:
  Mat3 m_K;
  Mat m_distort;

  int m_cols;
  int m_rows;
};

}  // namespace camera
#endif  // CAMERA_H
