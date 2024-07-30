/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

// Receive images via Redis and Protobuf

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include "../3rdParty/stlplus3/filesystemSimplified/file_system.hpp"
#include "slam/Config.h"
#include "slam/Parameters.h"
#include "slam/Converter.h"
#include "slam/System.h"
#include "slam/ViewPoint.h"

#include <sw/redis++/redis++.h>
#include "/home/xubo/Codes/slam/jointslam/build/poseInfo.pb.h"
#include "/home/xubo/Codes/slam/jointslam/build/imageInfo.pb.h"

NormalPrameter nprameter;

using namespace std;
using namespace sw::redis;

void RedisGetMessages(std::vector<std::string> channels);
void call_back(std::string channel, std::string msg);

void RedisG2L();
void g2l_call_back(std::string channel, std::string msg);

std::vector<double> vTimestamps;
std::vector<Vec3> map_abs_xyz;
std::vector<Qua> map_abs_qwxyz;
std::vector<double> conf;
std::vector<std::string> channels = {"server", "g2l"}; // server: 接收图像流; g2l：接收位姿信息
bool END = false;

ORB_SLAM2::System *slam;

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    cerr << endl
         << "Usage: ./vslam_test path_to_vocabulary path_to_settings "
            " path_output"
         << endl;
    return 1;
  }

  std::string sDbow = argv[1];
  std::string sSeting = argv[2];
  std::string sOutDir = argv[3];

  std::cout << "Start Visual-SLAM" << std::endl;

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(sDbow, sSeting, ORB_SLAM2::System::MONOCULAR, true);
  slam = &SLAM;

  nprameter.NP_Ready = false;
  nprameter.Num_Ready = 0;
  nprameter.Num_Need = 30;
  nprameter.wait = 0;
  nprameter.use = false;

  std::cout << "\n-------\n";
  std::cout << "Start processing sequence ..." << endl;

  // 接收图片和位姿
  RedisGetMessages(channels);
  cout << "-------\n\n";

  // Stop all threads
  SLAM.Shutdown();

  // save trajectory and map
  std::string sPath1 = stlplus::create_filespec(sOutDir, "KeyFrameTrajectory.txt");
  std::string sPath2 = stlplus::create_filespec(sOutDir, "CompleteTrajectory.txt");
  std::string sMapPath = stlplus::create_filespec(sOutDir, "MapPoints.ply");

  SLAM.SaveKeyFrameTrajectoryTUM(sPath1);
  SLAM.SaveTrajectoryTUM(sPath2);
  SLAM.SaveMapPoints(sMapPath);

  return 0;
}

void RedisGetMessages(std::vector<std::string> channels)
{
  std::cout << "Begin to listen ...\n";
  Redis redis("tcp://127.0.0.1:6379");
  auto sub = redis.subscriber();

  // sub.subscribe("server");
  sub.subscribe("g2l");

  sub.on_message(g2l_call_back);

  while (!END)
  {
    try
    {
      sub.consume();
    }
    catch (const Error &err)
    {
      // Handle exceptions.
    }
  }
  std::cout << "Finish listening ...\n";
}

void call_back(std::string channel, std::string msg)
{
  if (channel == "server")
  {
    camera::AbsXY abs_xy;
    imagebag::imageInfo image;
    clock_t startTime = clock();

    abs_xy.flag = true;
    image.ParseFromString(msg);
    float timestamp = atof(image.timestamp().c_str());

    std::vector<uchar> img_data(image.image_data().begin(), image.image_data().end());
    if (img_data.size() == 0)
    {
      printf("Empty image data!\n");
      return;
    }

    cv::Mat im = cv::imdecode(img_data, 0);
    slam->TrackMonocular(im, timestamp, abs_xy);

    double ttrack = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
    std::cout << "--Processing Speed: " << 1.0 / ttrack << " FPS" << std::endl;
    std::cout << "\n\n";

    if (timestamp == -1)
    {
      END = true;
    }
  }

  if (channel == "g2l")
  {
    posebag::poseInfo pos;
    pos.ParseFromString(msg);
    float x, y, z, qw, qx, qy, qz, c;
    x = pos.x();
    y = pos.y();
    z = pos.z();
    qw = pos.qw();
    qx = pos.qx();
    qy = pos.qy();
    qz = pos.qz();
    float timestamp = atof(pos.timestamp().c_str());
    c = pos.conf();

    nprameter.kfIDs.push_back(int(timestamp));
    nprameter.kfPos.push_back(Vec3(x, y, z));
    nprameter.kfOrin.push_back(Vec4(qw, qx, qy, qz));
    nprameter.kfConf.push_back(c);

    std::cout << "\n\nGet new key frame :" << int(timestamp) << "!\n\n";
  }
}

void g2l_call_back(std::string channel, std::string msg)
{
  if (channel == "g2l")
  {
    camera::AbsXY abs_xy;

    posebag::poseInfo pos;
    pos.ParseFromString(msg);

    float x, y, z, qw, qx, qy, qz, c;
    float timestamp = atof(pos.timestamp().c_str());
    x = pos.x();
    y = pos.y();
    z = pos.z();
    qw = pos.qw();
    qx = pos.qx();
    qy = pos.qy();
    qz = pos.qz();
    c = pos.conf();

    abs_xy.flag = (c != 0.0);
    abs_xy.xyz = Vec3(x, y, z);
    abs_xy.qwxyz = Vec4(qw, qx, qy, qz);
    abs_xy.sqrt_cov = Mat3::Identity() * c * 10;

    std::vector<uchar> img_data(pos.image_data().begin(), pos.image_data().end());
    if (img_data.size() == 0)
    {
      printf("Empty image data!\n");
      return;
    }

    if (timestamp == -1)
    {
      END = true;
      return;
    }
    std::cout << "\nGet timestamp: " << timestamp << " ...\n";
    cv::Mat im = cv::imdecode(img_data, 0);
    slam->TrackMonocular(im, timestamp, abs_xy);
  }
}