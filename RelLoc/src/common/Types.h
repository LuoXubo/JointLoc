#ifndef TYPES_H
#define TYPES_H

#include <map>
#include <vector>

#include <Eigen/Dense>

//////////////////////////////////////////////////////////////////////
#define NUMPARAMCAM 6      // the number of parameters for each camera
#define NUMGAUGEFREEDOM 7  // the freedom dimension of gauge
#define PTSDIM 3           // the dimension of points coordinate
#define MAX_VALUE 1E20
#define ANGLE_INTERSECTION_THRESHOLD \
  1.5  // too small intersection angle lead to unprecision

//////////////////////////////////////////////////////////////////////
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;

typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;

typedef Eigen::Quaterniond Qua;

typedef Eigen::Matrix<double, 2, 2> Mat2;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 4, 4> Mat4;
typedef Eigen::Matrix<double, 6, 6> Mat6;
typedef Eigen::Matrix<double, 9, 9> Mat9;
typedef Eigen::Matrix<double, 3, 4> Mat34;
typedef Eigen::Matrix<double, 2, 9> Mat29;
typedef Eigen::Matrix<double, 2, 6> Mat26;
typedef Eigen::Matrix<double, 7, 7> Mat7;

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Mat2X;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Mat3X;
typedef Eigen::Matrix<double, 4, Eigen::Dynamic> Mat4X;

typedef size_t
    CameraID;  // the ID of camera in each view -- according to configuration
typedef long unsigned int
    ViewID;  // the ID of view in image sequence, --keep growing
typedef long unsigned int TrackID;  // the ID of 3D tie points --keep growing
typedef long unsigned int
    FrameID;  // the ID of view in image sequence, --keep growing

// for feature points
typedef int FeatID;
typedef int ClassID;
typedef int BucketID;

//////////////////////////////////////////////////////////////////////
// defination of landmark
typedef int RSID;
typedef int RMID;
typedef size_t VertexID;

typedef std::pair<RSID, VertexID> RSINDEX;  // <RS_ID, vertex>

typedef std::vector<RSID> RSIDs;
typedef std::pair<Vec3, Vec3> LineSegment3D;
typedef std::vector<LineSegment3D> LineSegment3Ds;
typedef std::vector<Vec3> Polyline3;
typedef std::vector<Vec2> Polyline2;

//////////////////////////////////////////////////////////////////////

enum GEO_TYPE  // data type
{ ROAD_SIGN,
  ROAD_MARK,
  ROAD_SIGN_MARK };

enum CAM_CONFIG_TYPE  // type of camera configuration
{ MONO = 1,           // monocular
  F_F,                // forward stereo
  F_B,                // forward-backward stereo
  F_F_B_B             // four camera
};

enum FILTER_METHOD  // geometric method
{ F_MATRIX,         // fundamental
  H_MATRIX          // homography
};

//////////////////////////////////////////////////////////////////////
// retrive the keys in map
struct RetrieveKey {
  template <typename T>
  typename T::first_type operator()(T p) const {
    return p.first;
  }
};

struct RetrieveValue {
  template <typename T>
  typename T::second_type operator()(const T &p) const {
    return p.second;
  }
};

struct GPSData {
  double timestamp;
  double latitude;
  double longitude;
  double altitude;
  double heading;
  double pitch;
  double roll;
};
typedef std::vector<GPSData> GPSDataVec;

struct SettingParams {
  size_t fps;  // sequence frequency
  bool bRGB;
  float bf;              // length of baseline
  float thDepth;         // depth
  float depthMapFactor;  // for RGBD slam
};

struct ORBParams  // ORB Feature detection param
{
  int numFeatures;
  float scaleFactor;
  int numLevels;
  int numIniThFAST;  // init FAST threshold
  int numMinThFAST;  // min FAST threshold
};

struct SLAMConfig {
  SettingParams settingParam;
  ORBParams orbFeatParam;
};

#endif  // TYPES_H
