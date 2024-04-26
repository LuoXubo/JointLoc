#ifndef MODEL_LBA_ERROR_H
#define MODEL_LBA_ERROR_H

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "common/Types.h"
#include "sim3.h"
extern NormalPrameter nprameter;

namespace BA
{

  template <typename T1, typename T2>
  std::vector<T1> RetriveID(const std::map<T1, T2> &m)
  {
    std::vector<T1> vecKeys;
    std::transform(m.begin(), m.end(), std::back_inserter(vecKeys),
                   RetrieveKey());
    return vecKeys;
  }

  /*all the perameters in LBA
   * The order is:
   * [poses]---[3D tie points]---[camera2view] ----[GCPs]
   */

  class BAParameters
  {
  public:
    static const size_t POSE_BLOCK_SIZE = 6; // number of parameters for each pose
    static const size_t PT_BLOCK_SIZE = 3;   // X ,Y, Z

    explicit BAParameters() {}
    ~BAParameters()
    {
      m_mapPtsIdx.clear();
      m_mapViewIdx.clear();
      m_vecParam.clear();
    }

    size_t num_params() const { return m_vecParam.size(); }
    size_t num_views() const { return m_mapViewIdx.size(); }
    size_t num_points() const { return m_mapPtsIdx.size(); }
    size_t num_cams() const { return m_mapCameraIdx.size(); }
    size_t num_GCPs() const { return m_mapGCPIdx.size(); }
    size_t num_view2IMU() const { return m_mapView2IMUIdx.size(); }

    // exist
    bool view_exist(ViewID id) const
    {
      return (m_mapViewIdx.find(id) == m_mapViewIdx.end() ? false : true);
    }
    bool view_fixed(ViewID id) const { return m_mapViewFixed.at(id); }
    bool trk_exist(TrackID id) const
    {
      return (m_mapPtsIdx.find(id) == m_mapPtsIdx.end() ? false : true);
    }
    bool cam_exist(CameraID id) const
    {
      return (m_mapCameraIdx.find(id) == m_mapCameraIdx.end() ? false : true);
    }
    bool gcp_exist(RSINDEX id) const
    {
      return (m_mapGCPIdx.find(id) == m_mapGCPIdx.end() ? false : true);
    }

    // get parameter block by ID
    double *mutable_view(ViewID id)
    {
      return mutable_views() + idx_view(id) * POSE_BLOCK_SIZE;
    }
    double *mutable_point(TrackID id)
    {
      return mutable_points() + idx_trk(id) * PT_BLOCK_SIZE;
    }
    double *mutable_cam(CameraID id)
    {
      return mutable_cams() + idx_cam(id) * POSE_BLOCK_SIZE;
    }
    double *mutable_GCP(RSINDEX id)
    {
      return mutable_GCPs() + idx_gcp(id) * PT_BLOCK_SIZE;
    }
    double *mutable_vp2IMU() { return mutable_view2IMU(); }

    bool isViewFixed(ViewID id) { return m_mapViewFixed.at(id); }
    bool isPointFixed(TrackID id) { return m_mapPtsFixed.at(id); }

    // seperate rotation and translation for pose
    double *mutable_view_rotation(ViewID id) { return mutable_view(id) + 3; }
    double *mutable_view_position(ViewID id) { return mutable_view(id); }

    double *mutable_cam_rotation(CameraID id) { return mutable_cam(id) + 3; }
    double *mutable_cam_position(CameraID id) { return mutable_cam(id); }

    double *mutable_viewimu_rotation() { return mutable_view2IMU() + 3; }
    double *mutable_viewimu_position() { return mutable_view2IMU(); }

    // input
    void param_push_back(double a) { m_vecParam.push_back(a); }
    void viwIdx_push_back(ViewID id, size_t idx) { m_mapViewIdx[id] = idx; }
    void ptsIdx_push_back(TrackID id, size_t idx) { m_mapPtsIdx[id] = idx; }
    void camIdx_push_back(CameraID id, size_t idx) { m_mapCameraIdx[id] = idx; }
    void gcpIdx_push_back(RSINDEX id, size_t idx) { m_mapGCPIdx[id] = idx; }
    void view2IMUIdx_push_back(size_t id, size_t idx)
    {
      m_mapView2IMUIdx[id] = idx;
    }

    // Fixed or not
    void viwFixedFlag_push_back(ViewID id, bool falg)
    {
      m_mapViewFixed[id] = falg;
    }
    void ptsFixedFlag_push_back(TrackID id, bool falg)
    {
      m_mapPtsFixed[id] = falg;
    }

    // retrive
    const std::vector<ViewID> getViews() { return RetriveID(m_mapViewIdx); }
    const std::vector<TrackID> get3DPoints() { return RetriveID(m_mapPtsIdx); }
    const std::vector<CameraID> getCameras() { return RetriveID(m_mapCameraIdx); }
    const std::vector<RSINDEX> getGCPs() { return RetriveID(m_mapGCPIdx); }

  protected:
    // mutable blocks of parameters
    double *mutable_views() { return &m_vecParam[0]; }
    double *mutable_points()
    {
      return mutable_views() + num_views() * POSE_BLOCK_SIZE;
    }
    double *mutable_cams()
    {
      return mutable_points() + num_points() * PT_BLOCK_SIZE;
    }
    double *mutable_GCPs()
    {
      return mutable_cams() + num_cams() * POSE_BLOCK_SIZE;
    }
    double *mutable_view2IMU()
    {
      return mutable_GCPs() + num_GCPs() * PT_BLOCK_SIZE;
    }

    const size_t idx_view(ViewID id) const { return m_mapViewIdx.at(id); }
    const size_t idx_trk(TrackID id) const { return m_mapPtsIdx.at(id); }
    const size_t idx_cam(CameraID id) const { return m_mapCameraIdx.at(id); }
    const size_t idx_gcp(RSINDEX id) const { return m_mapGCPIdx.at(id); }

  private:
    std::map<ViewID, size_t> m_mapViewIdx; // poses
    std::map<ViewID, bool> m_mapViewFixed;

    std::map<TrackID, size_t> m_mapPtsIdx; // pts3d
    std::map<TrackID, bool> m_mapPtsFixed;

    std::map<CameraID, size_t> m_mapCameraIdx; // camera Id
    std::map<RSINDEX, size_t> m_mapGCPIdx;
    std::map<size_t, size_t> m_mapView2IMUIdx;

    std::vector<double> m_vecParam;
  };

  /////////////////////////////////////////////////////////////////////////////////////
  /// \brief The ReprojectionError class
  ///
  class ReprojectionError
  {
  public:
    ReprojectionError(double observed_x, double observed_y, double focal,
                      double s)
        : m_observed_x(observed_x), m_observed_y(observed_y), m_f(focal), m_w(s)
    {
    }

    ~ReprojectionError() {}
    /// Compute the residual error after reprojection
    /// residual = observed - euclidean( focal * [R|C] point)

    template <typename T>
    bool operator()(const T *const pointer_x, const T *const pointer_y,
                    const T *const pointer_z, // position
                    const T *const pointer_r1, const T *const pointer_r2,
                    const T *const pointer_r3, // angle axis
                    const T *const point, T *residuals) const
    {
      typedef Eigen::Matrix<T, 3, 1> Vec3;

      T angle[3];
      angle[0] = *pointer_r1, angle[1] = *pointer_r2, angle[2] = *pointer_r3;
      Vec3 p;
      ceres::AngleAxisRotatePoint(angle, point, p.data());

      T C[3];
      C[0] = *pointer_x, C[1] = *pointer_y, C[2] = *pointer_z;
      Vec3 t;
      ceres::AngleAxisRotatePoint(angle, C, t.data());

      p -= t;
      T xp = p[0] / p[2];
      T yp = p[1] / p[2];

      T predicted_x = m_f * xp;
      T predicted_y = m_f * yp;
      residuals[0] = predicted_x - T(m_observed_x);
      residuals[1] = predicted_y - T(m_observed_y);
      residuals[0] *= T(m_w);
      residuals[1] *= T(m_w);

      // 一开始没加z的约束
      // T predicted_z = m_f * zp;
      // residuals[2] = predicted_z - T(m_observed_z);
      // residuals[2] *= T(m_w);
      return true;
    }

    /////////////////////////////////////////////////////////////////////
    // other cameras, relative to camera 0
    template <typename T>
    bool operator()(
        const T *const pointer_x, const T *const pointer_y,
        const T *const pointer_z, // position
        const T *const pointer_r1, const T *const pointer_r2,
        const T *const pointer_r3, //
        const T *const point,
        const T *const trans, // relative parameters from camera to view point
        const T *const rot, T *residuals) const
    {
      typedef Eigen::Matrix<T, 3, 3> Mat3T;
      typedef Eigen::Matrix<T, 3, 1> Vec3T;

      Mat3T camR, viewR;

      T angle[3];
      angle[0] = *pointer_r1, angle[1] = *pointer_r2, angle[2] = *pointer_r3;

      ceres::AngleAxisToRotationMatrix(angle, viewR.data());

      ceres::AngleAxisToRotationMatrix(rot, camR.data());
      Mat3T camPose_R = camR * viewR; // rotation matrix

      Vec3T viewC(*pointer_x, *pointer_y, *pointer_z);
      Vec3T camOffset(trans[0], trans[1], trans[2]);

      // translation of camera
      // Vec3T camPose_t = -camPose_R*(viewC + viewR.transpose()*camOffset);

      Vec3T camPose_t = -camPose_R * viewC + camOffset;

      Vec3T p(point[0], point[1], point[2]);

      p = camPose_R * p + camPose_t;

      T xp = p(0, 0) / p(2, 0);
      T yp = p(1, 0) / p(2, 0);

      T predicted_x = m_f * xp;
      T predicted_y = m_f * yp;

      residuals[0] = predicted_x - T(m_observed_x);
      residuals[1] = predicted_y - T(m_observed_y);
      residuals[0] *= T(m_w);
      residuals[1] *= T(m_w);

      // std::cout << "res: " << residuals[0] << "," << residuals[1] << std::endl;
      return true;
    }

    void computeErrorAndDepth(const double *const view, const double *const point,
                              double &res, double &depth)
    {
      Mat3 R;
      ceres::AngleAxisToRotationMatrix(view + 3, R.data());
      Vec3 C(view[0], view[1], view[2]);

      Vec3 p(point[0], point[1], point[2]);
      Vec3 pTmp = R * (p - C);
      depth = pTmp[2];

      Vec2 predict = pTmp.head(2) / pTmp[2] * m_f;
      Vec2 obs(m_observed_x, m_observed_y);
      Vec2 tmp = (predict - obs) * m_w;
      res = tmp.norm();
    }

    static ceres::CostFunction *create(double obs_x, double obs_y, double f,
                                       double invSigma)
    {
      return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 1, 1, 1, 1, 1,
                                             1, 3>(
          new ReprojectionError(obs_x, obs_y, f, invSigma));
    }

  private:
    double m_observed_x;
    double m_observed_y;
    double m_f; // focal
    double m_w; // weight
  };

  /////////////////////////////////////////////////////////////////////////////////////
  /// \brief The ReprojectionError class for pose only optimization
  ///
  class ReprojectionErrorPoseOnly
  {
  public:
    ReprojectionErrorPoseOnly(const Vec2 &imPts, const Vec3 &mpPts, double focal,
                              double s)
        : m_imPts(imPts), m_mpPts(mpPts), m_f(focal), m_w(s) {}

    ~ReprojectionErrorPoseOnly() {}

    template <typename T>
    bool operator()(const T *const pointer_x, const T *const pointer_y,
                    const T *const pointer_z, const T *const pointer_r1,
                    const T *const pointer_r2, const T *const pointer_r3, //
                    T *residuals) const
    {
      typedef Eigen::Matrix<T, 3, 1> Vec3T;

      T angle[3];
      angle[0] = *pointer_r1, angle[1] = *pointer_r2, angle[2] = *pointer_r3;
      Vec3T pcw = m_mpPts.template cast<T>();
      Vec3T p;
      ceres::AngleAxisRotatePoint(angle, pcw.data(), p.data());

      T C[3];
      C[0] = *pointer_x, C[1] = *pointer_y, C[2] = *pointer_z;
      Vec3T t;
      ceres::AngleAxisRotatePoint(angle, C, t.data());

      p -= t;

      T predicted_x = m_f * p[0] / p[2];
      T predicted_y = m_f * p[1] / p[2];
      residuals[0] = m_w * (predicted_x - T(m_imPts[0]));
      residuals[1] = m_w * (predicted_y - T(m_imPts[1]));

      return true;
    }

    static ceres::CostFunction *create(const Vec2 &obs, const Vec3 &pt3D,
                                       double f, double invSigma)
    {
      return new ceres::AutoDiffCostFunction<ReprojectionErrorPoseOnly, 2, 1, 1,
                                             1, 1, 1, 1>(
          new ReprojectionErrorPoseOnly(obs, pt3D, f, invSigma));
    }

    static Vec2 back_project(const Vec2 &obs, const Vec3 &pt3D, const Vec6 &pose,
                             double f)
    {
      Vec3 vAngle = pose.tail<3>();
      Vec3 vPos = pose.head<3>();

      Vec3 p, t;
      ceres::AngleAxisRotatePoint(vAngle.data(), pt3D.data(), p.data());
      ceres::AngleAxisRotatePoint(vAngle.data(), vPos.data(), t.data());
      p -= t;
      return (obs - f * p.head<2>() / p[2]);
    }

  private:
    Vec2 m_imPts;
    Vec3 m_mpPts;
    double m_f; // focal
    double m_w; // weight
  };

  // Class for absolute XYZ constraint.
  class PositionXYError
  {
  public:
    PositionXYError(const ceres::Vector abs_xy, const ceres::Matrix xy_w, ceres::Matrix R, ceres::Vector t, double s)
        : m_abs_xy(abs_xy), m_xy_w(xy_w), m_R(R), m_t(t), m_s(s) {}

    template <typename T>
    bool operator()(const T *const X, const T *const Y, const T *const Z, T *res) const
    {
      typedef Eigen::Matrix<T, 3, 1> Vec3;

      Vec3 C(*X, *Y, *Z);

      Vec3 Cc = m_R.template cast<T>() * C;
      Cc = m_s * Cc;
      Cc = m_t.template cast<T>() + Cc;

      Vec3 vec_res = m_abs_xy.template cast<T>() - Cc;
      vec_res = m_xy_w.template cast<T>() * vec_res;

      res[0] = vec_res[0];
      res[1] = vec_res[1];
      res[2] = vec_res[2];

      return true;
    }

    static ceres::CostFunction *create(const Vec3 &abs_xy, const Mat3 &xy_w, Mat3 R, Vec3 t, double s)
    {
      return new ceres::AutoDiffCostFunction<PositionXYError, 3, 1, 1, 1>(
          new PositionXYError(abs_xy, xy_w, R, t, s));
    }

  private:
    ceres::Vector m_abs_xy;
    ceres::Matrix m_xy_w;
    ceres::Matrix m_R;
    ceres::Vector m_t;
    double m_s;
  };

  // Class for GPS/IMU (XYZ/RPY) constraints.
  class GPSIMUError
  {
  public:
    static ceres::CostFunction *create(const Vec3 &Owi, const Mat3 &Rwi,
                                       const Mat6 &sqrtW)
    {
      return new ceres::AutoDiffCostFunction<GPSIMUError, 6, 1, 1, 1, 1, 1, 1, 6>(
          new GPSIMUError(Owi, Rwi, sqrtW));
    }

    static ceres::CostFunction *create_pose_cost(const Vec3 &Ow, const Mat3 &Rcw,
                                                 const Mat6 &sqrtW)
    {
      return new ceres::AutoDiffCostFunction<GPSIMUError, 6, 1, 1, 1, 1, 1, 1>(
          new GPSIMUError(Ow, Rcw, sqrtW));
    }

    GPSIMUError(const Vec3 &C, const Mat3 &R, const Mat6 &wSqrt)
        : m_iO(C), m_iRwi(R), m_calibRci(Mat3::Identity()),
          m_calibTci(Vec3(0, 0, 0)), m_matWSqrt(wSqrt) {}

    GPSIMUError(const Vec3 &Cwi, const Mat3 &Rwi, const Vec3 &Tci,
                const Mat3 &Rci, const Mat6 &wSqrt)
        : m_iO(Cwi), m_iRwi(Rwi), m_calibRci(Rci), m_calibTci(Tci),
          m_matWSqrt(wSqrt) {}

    ~GPSIMUError() {}

    template <typename T>
    bool operator()(const T *const X, const T *const Y, const T *const Z,
                    const T *const r1, const T *const r2, const T *const r3,
                    const T *const Tci, // transformation from imu to view
                    T *res) const
    {
      typedef Eigen::Matrix<T, 3, 3> Mat3T;
      typedef Eigen::Matrix<T, 3, 1> Vec3T;

      T angle[3];
      angle[0] = *r1;
      angle[1] = *r2;
      angle[2] = *r3;

      Mat3T Rc, Rci;
      ceres::AngleAxisToRotationMatrix(angle, Rc.data());
      ceres::AngleAxisToRotationMatrix(Tci + 3, Rci.data());

      Mat3T Rcw = Rci * (m_iRwi.template cast<T>()).transpose();

      Vec3T tci(Tci[0], Tci[1], Tci[2]);
      Vec3T C = m_iO.template cast<T>() - Rcw.transpose() * tci;

      Mat3T matRes = Rc * Rcw.transpose();
      Eigen::Quaternion<T> deltaQ(matRes);

      Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(res);
      residuals.template block<3, 1>(0, 0) = Vec3T(*X, *Y, *Z) - C;
      residuals.template block<3, 1>(3, 0) = T(2.0) * deltaQ.vec();

      residuals.applyOnTheLeft(m_matWSqrt.template cast<T>());

      return true;
    }
    template <typename T>
    bool operator()(const T *const X, const T *const Y, const T *const Z,
                    const T *const r1, const T *const r2, const T *const r3,
                    T *res) const
    {
      typedef Eigen::Matrix<T, 3, 3> Mat3T;
      typedef Eigen::Matrix<T, 3, 1> Vec3T;

      T angle[3];
      angle[0] = *r1;
      angle[1] = *r2;
      angle[2] = *r3;

      Mat3T Rc;
      ceres::AngleAxisToRotationMatrix(angle, Rc.data());

      Mat3T Rcw = m_iRwi.template cast<T>();
      Vec3T C = m_iO.template cast<T>();

      Mat3T matRes = Rc * Rcw.transpose();
      Eigen::Quaternion<T> deltaQ(matRes);
      Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(res);
      residuals.template block<3, 1>(0, 0) = Vec3T(*X, *Y, *Z) - C;
      residuals.template block<3, 1>(3, 0) = T(2.0) * deltaQ.vec();

      residuals.applyOnTheLeft(m_matWSqrt.template cast<T>());

      return true;
    }

  private:
    Vec3 m_iO;   // absolute position from IMU
    Mat3 m_iRwi; // imu to world R
    Mat3 m_calibRci;
    Vec3 m_calibTci;
    Mat6 m_matWSqrt;
  };

} // namespace BA

#endif //
