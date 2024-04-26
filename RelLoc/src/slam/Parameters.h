#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "common/Types.h"

// define some threshold values used in SLAM
namespace Param {
// image and map points
//-----------------------------------------
const float TH_CHI_2D = sqrt(5.991);
const float TH_CHI_3D = sqrt(7.815);
//-----------------------------------------

// Tracking
//-----------------------------------------
const size_t MIN_PAIRWISE_MATCHES_NUMBER = 15;
const size_t MIN_2D3D_MATCHES_NUMBER = 15;
const float ANGLE_INTERSECTION_COS = 0.99984769515;             // cos(1 deg)
const float ANGLE_INTERSECTION_COS_LOWERBOUND = 0.99862953475;  // cos(3.0 deg)
const float ANGLE_INTERSECTION_COS_UPPERBOUND = 0.99999390765;  // cos(0.2 deg)

//-----------------------------------------

// BA
//-----------------------------------------
const size_t MAX_THREAD_FOR_SOLVER = 4;
const size_t MAX_ITERATION_NUMBER = 15;
const float MAX_SLOVER_TIME = 5;  // 0.15s

//-----------------------------------------

// GPS, IMU ...
//-----------------------------------------
const float MAX_OFFSET_FOR_GPS_POINT = 20;                // 20 meters
const float SIGMA_GPS_POINT_NOISE = 1.0;                  // 1.0 meter
const float SIGMA_IMU_ROTATION_NOISE = 1.0 / 180 * M_PI;  // 1.0 degree
const float LAMBA_REGULARIZATION_FACTOR = 150;
// if the interval is larger than 2.5m, the frame is a key frame
const float TH_KEYFRAME_INTERVAL = 2.5;
//-----------------------------------------

// CAM IMU Calibration
//-----------------------------------------
const float SIGMA_TRANSLATION_CAM_IMU = 0.01;
const float SIGMA_ROTATION_CAM_IMU = 1.0 / 180 * M_PI;
//-----------------------------------------
}  // namespace Param

#endif  // PARAMETERS_H
