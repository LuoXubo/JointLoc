#ifndef SIM_3
#define SIM_3

#include <ceres/rotation.h>

#include "common/Types.h"

class Sim3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  Sim3() {
    m_r.setIdentity();
    m_t.fill(0.);
    m_s = 1.;
  }

  Sim3(const Eigen::Quaterniond &r, const Vec3 &t, double s)
      : m_r(r), m_t(t), m_s(s) {}
  Sim3(const Mat3 &R, const Vec3 &t, double s)
      : m_r(Eigen::Quaterniond(R)), m_t(t), m_s(s) {}

  Sim3(const Vec7 &update)  // angleaxis, transformation, scale
  {
    Vec3 angleAxis;
    for (int i = 0; i < 3; i++) angleAxis[i] = update[i];

    Mat3 R = Mat3::Identity();
    ceres::AngleAxisToRotationMatrix(angleAxis.data(), R.data());
    m_r = Eigen::Quaterniond(R);
    for (int i = 0; i < 3; i++) m_t[i] = update[i + 3];
    m_s = update[6];
  }
  Vec7 get7DoFSim3() const {
    Vec7 res;
    Mat3 R = m_r.toRotationMatrix();
    Vec3 angleAxis;
    ceres::RotationMatrixToAngleAxis(R.data(), angleAxis.data());
    for (int i = 0; i < 3; i++) res[i] = angleAxis[i];

    for (int i = 0; i < 3; i++) res[i + 3] = m_t[i];
    res[6] = m_s;
    return res;
  }

  Vec3 map(const Vec3 &xyz) const {
    // Mat3 R = m_r.toRotationMatrix();
    return m_s * (m_r * xyz) + m_t;
  }

  Sim3 inverse() const {
    return Sim3(m_r.conjugate(), m_r.conjugate() * ((-1. / m_s) * m_t),
                1. / m_s);
  }

  Sim3 operator*(const Sim3 &other) const {
    Sim3 ret;
    ret.m_r = m_r * other.m_r;
    ret.m_t = m_s * (m_r * other.m_t) + m_t;
    ret.m_s = m_s * other.m_s;
    return ret;
  }

  Sim3 &operator*=(const Sim3 &other) {
    Sim3 ret = (*this) * other;
    *this = ret;
    return *this;
  }

  inline const Vec3 &translation() const { return m_t; }

  inline Vec3 &translation() { return m_t; }

  inline const Eigen::Quaterniond &rotation() const { return m_r; }

  inline Eigen::Quaterniond &rotation() { return m_r; }

  inline const double &scale() const { return m_s; }

  inline double &scale() { return m_s; }

 protected:
  Eigen::Quaterniond m_r;
  Vec3 m_t;
  double m_s;
};

inline std::ostream &operator<<(std::ostream &out_str, const Sim3 &sim3) {
  out_str << sim3.rotation().coeffs() << std::endl;
  out_str << sim3.translation() << std::endl;
  out_str << sim3.scale() << std::endl;

  return out_str;
}

#endif
