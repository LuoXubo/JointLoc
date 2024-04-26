#include <sw/redis++/redis++.h>
#include "/home/xubo/Codes/slam/jointslam/build/imageInfo.pb.h"
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace sw::redis;

void PubImages(string filepath);

int main(int argc, char **argv)
{
    std::string sInDir = argv[1];
    if (argc != 2)
    {
        cerr << endl
             << "Usage: ./server path_to_images"
             << endl;
        return 1;
    }
    PubImages(sInDir);

    std::cout << "Finish publishing ...\n";
    return 0;
}

void PubImages(std::string filepath)
{
    std::cout << "Begin to publish ...\n";
    Redis redis("tcp://127.0.0.1:6379");
    std::string strFile = filepath + "/images.txt";
    ifstream f;
    f.open(strFile.c_str());
    cv::Mat im;
    std::vector<unsigned char> image_data;
    imagebag::imageInfo msg;

    // skip first three lines
    std::string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof())
    {
        std::string s;
        getline(f, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            std::string t;
            std::string sRGB;
            ss >> t;
            ss >> sRGB;

            im = cv::imread(filepath + sRGB);
            image_data.assign(im.data, im.data + im.total() + im.elemSize());

            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at: "
                     << filepath + sRGB << endl;
                return;
            }

            msg.set_timestamp(t);
            msg.set_image_data(image_data.data(), image_data.size());
            std::string sas_string = msg.SerializeAsString();
            redis.publish("source", sas_string);
        }
    }
}