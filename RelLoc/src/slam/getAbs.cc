#include "getAbs.h"
#include "ViewPoint.h"

#include <sw/redis++/redis++.h>
#include "/home/xubo/Codes/slam/jointslam/build/poseInfo.pb.h"

extern NormalPrameter nprameter;

namespace ORB_SLAM2
{
    getAbs::getAbs() : {}

    void getAbs::Run()
    {
        sw::redis::Redis redis("tcp://127.0.0.1:6379");
        auto sub = redis.subscriber();
        sub.subscribe("g2l");
        sub.on_message(call_back);

        while (1)
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

    void getAbs::call_back(std::string channel, std::string msg)
    {
        if (channel == "g2l")
        {
            posebag::poseInfo pose;
            pose.ParseFromString(msg);
            nprameter.kfIDs.push_back(int(atof(pose.timestamp().c_str())));
            nprameter.kfConf.push_back(pose.conf());

            float x, y, z, qw, qx, qy, qz;
            x = pose.x();
            y = pose.y();
            z = pose.z();
            qw = pose.qw();
            qx = pose.qx();
            qy = pose.qy();
            qz = pose.qz();

            nprameter.kfPos.push_back(Vec3(x, y, z));
            nprameter.kfOrin.push_back(Vec4(qw, qx, qy, qz));

            std::cout << "Get new key frame: " << pose.timestamp() << "!\n\n";
        }
    }
}
