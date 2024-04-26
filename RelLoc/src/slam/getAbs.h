#ifndef GETABS_H
#define GETABS_H

#include <mutex>
#include <stdlib>
#include <sw/redis++/redis++.h>

#include "Initializer.h"
#include "Map.h"
#include "Tracking.h"

#include "/home/xubo/Codes/slam/jointslam/build/poseInfo.pb.h"

NormalPrameter nprameter;

namespace ORB_SLAM2
{
    class getAbs
    {
    public:
        getAbs();

        // Main function
        void Run();
        void call_back(std::string channel, std::string msg);
    }
}