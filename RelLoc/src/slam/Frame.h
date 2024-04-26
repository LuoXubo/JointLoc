#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "DBoWHeader.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "ViewPoint.h"

namespace ORB_SLAM2 {
// #define FRAME_GRID_ROWS 48
// #define FRAME_GRID_COLS 64
#define FRAME_GRID_ROWS 34
#define FRAME_GRID_COLS 62

class MapPoint;
class KeyFrame;

class Frame {
public:
  Frame();
  Frame(const Frame &frame);

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth);

  // Constructor for stereo cameras.
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
        ORBextractor *extractorLeft, ORBextractor *extractorRight,
        ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf,
        const float &thDepth);

  // Constructor for RGB-D cameras.
  Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
        ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K,
        cv::Mat &distCoef, const float &bf, const float &thDepth);

  // Extract ORB on the image. 0 for left image and 1 for right image.
  void ExtractORB(int flag, const cv::Mat &im);

  // Compute Bag of Words representation.
  void ComputeBoW();

  // Set/get the camera pose.
  void SetPose(const cv::Mat &Tcw) { mPose = camera::Pose(Tcw); }
  const camera::Pose &GetPose() const { return mPose; }

  // Set / get abs xy
  void SetAbsXY(const camera::AbsXY &abs_xy) { mAbsXY = abs_xy; }
  const camera::AbsXY &GetAbsXY() const { return mAbsXY; }
  
//   // Set / get NPrameter
//   void SetNPrameter(const camera::NPrameter &nprameter) { mNPrameter = nprameter; }
//   const camera::NPrameter &GetNPrameter() const { return mNPrameter; }

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

  // Compute the cell of a keypoint (return false if outside the grid)
  bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

  vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                   const float &r, const int minLevel = -1,
                                   const int maxLevel = -1) const;

  // Search a match for each keypoint in the left image to a keypoint in the
  // right image. If there is a match, depth is computed and the right
  // coordinate associated to the left keypoint is stored.
  void ComputeStereoMatches();

  // Associate a "right" coordinate to a keypoint if there is valid depth in the
  // depthmap.
  void ComputeStereoFromRGBD(const cv::Mat &imDepth);

  // Backprojects a keypoint (if stereo/depth info available) into 3D world
  // coordinates.
  cv::Mat UnprojectStereo(const int &i);

public:
  // Vocabulary used for relocalization.
  ORBVocabulary *mpORBvocabulary;

  // Feature extractor. The right is used only in the stereo case.
  ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

  // Frame timestamp.
  double mTimeStamp;

  // Calibration matrix and OpenCV distortion parameters.
  cv::Mat mK;
  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;
  cv::Mat mDistCoef;

  int mImCols;
  int mImRows;

  // Stereo baseline multiplied by fx.
  float mbf;

  // Stereo baseline in meters.
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view.
  // Far points are inserted as in the monocular case from 2 views.
  float mThDepth;

  // Number of KeyPoints.
  int mNumOfKPts;

  // Vector of keypoints (original for visualization) and undistorted (actually
  // used by the system). In the stereo case, mvKeysUn is redundant as images
  // must be rectified. In the RGB-D case, RGB images can be distorted.
  std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
  std::vector<cv::KeyPoint> mvKeysUn;

  // Corresponding stereo coordinate and depth for each keypoint.
  // "Monocular" keypoints have a negative value.
  std::vector<float> mvuRight;
  std::vector<float> mvDepth;

  // Bag of Words Vector structures.
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // ORB descriptor, each row associated to a keypoint.
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints, NULL pointer if no association.
  std::vector<MapPoint *> mvpMapPoints;

  // Flag to identify outlier associations.
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity
  // when projecting MapPoints.
  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;

  // Scale pyramid info.
  int mnScaleLevels;
  float mfScaleFactor;
  float mfLogScaleFactor;
  vector<float> mvScaleFactors;
  vector<float> mvInvScaleFactors;
  vector<float> mvLevelSigma2;
  vector<float> mvInvLevelSigma2;

  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;

private:
  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).
  void UndistortKeyPoints();

  // Computes image bounds for the undistorted image (called in the
  // constructor).
  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the
  // constructor).
  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  // Camera pose.
  camera::Pose mPose;
  camera::AbsXY mAbsXY; 
//   camera::NPrameter mNPrameter; 
};

} // namespace ORB_SLAM2

#endif // FRAME_H
