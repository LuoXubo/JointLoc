#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <random>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../3rdParty/stlplus3/filesystemSimplified/file_system.hpp"
#include "slam/Config.h"
#include "slam/Parameters.h"
#include "slam/Converter.h"
#include "slam/System.h"


using namespace std;
using namespace cv;


void LoadImagesMars(const string &sImgDir,                  // 输入：图像序列所在文件夹
                    vector<string> &vstrImageFilenames);    // 输出：每张图像的路径

void LoadLocation(const string &strPathToSequence,          // 输入：高度图序列轨迹数据路径
                  vector<Vec3> &vstrtrajectoryFilenames,    // 输出：每张高度图对应相机的x,y,z坐标
                  vector<Vec4> &vstrquaterniondFilenames);  // 输出：每张高度图对应相机的四元数qw,qx,qy,qz

int main(int argc, char **argv)
{
    if (argc != 5) {
        cerr << "Usage: ./ReTMacher, path_to_HeightImage, path_to_OpticalImage, path_to_Translat" << endl;
        return 1;
    }
    
    // 输入：高位图序列路径
    std::string HImgPath = argv[1];
    // 输入：光学图序列路径
    std::string OImgPath = argv[2];
    // 输入：高位序列轨迹数据路径
    std::string HeightData = argv[3];
    // 输出：光学序列轨迹数据路径
    std::string OpticalData = argv[4];
    
    // 加载高位图像:HImg, 光学图像:OImg, 高位序列轨迹数据:Height_xyh
    vector<string> HImg;
    LoadImagesMars(HImgPath, HImg);
    
    vector<string> OImg;
    LoadImagesMars(OImgPath, OImg);
    
    std::vector<Vec3> ref_xyh;
    std::vector<Vec4> ref_qwxyz;
    LoadLocation(HeightData, ref_xyh, ref_qwxyz);
    
    // 建立输出结果流
    ofstream f;
    f.open(OpticalData.c_str());
    f<< fixed;
    
    // 进入主循环
    cv::Mat im1, im2;                   // 初始化高位图：im1, 光学图：im2
    cv::Mat dst1, dst2;                 // 将im1, im2分别转化成32位浮点型，用于计算相位相关
    std::vector<Vec3> Optical_xyh;      // 初始化光学图像序列轨迹数据
    int nImages = OImg.size();          // 统计光学图像序列总数
    
    // 给Z坐标施加高斯白噪声误差
    const double mean = 0.0;//均值
    const double stddev = 0.01;//标准差
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    
    
    for (int ni = 0; ni < nImages; ni++) 
    {
        // 读图片 
        im1 = cv::imread(HImg[ni], CV_LOAD_IMAGE_UNCHANGED);
        im2 = cv::imread(OImg[ni], CV_LOAD_IMAGE_UNCHANGED);

        // 判断图像是否存在
        std::cout << " load image " << ni << std::endl;
        if (im1.empty() || im2.empty()) 
        {
            cerr << endl<< "Failed to load image at: " << HImg[ni] << " or " << OImg[ni] <<endl;
            return 1;
        }

        // 计算偏移前对图像进行预处理
        
        cv::cvtColor( im1, im1, CV_BGR2GRAY );
        im1.convertTo(dst1, CV_32FC1);
        
        cv::cvtColor( im2, im2, CV_BGR2GRAY );
        im2.convertTo(dst2, CV_32FC1);
        
       
        clock_t startTime = clock();
        // 计算偏移结果
        double response;                                                         //相位相关响应值
        cv::Point2d offset=cv::phaseCorrelate(dst1, dst2, noArray(), &response);//相位相关偏移结果
        
        double ttrack = static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;

        std::cout << "--Processing Speed: " << 1000*ttrack << " ms" << std::endl << std::endl;

        // 计算分辨率，用于计算光学图像偏移值
        //                                                   视野范围角度                             
        // resolution = 2*(海拔高度 - 视野范围地面最低高度）*tan( -----------)/图像大小
        //                                                       2
        //
        double minPixl;                                     // 初始化像素最小值
        cv::Point  minIdx;                                  // 像素最小值坐标    
        cv::minMaxLoc(im1, &minPixl, NULL, &minIdx, NULL);  // 求取
        
        
        uchar *data=im1.ptr<uchar>(239);
        int cur =data[239];
        
        double minh=minPixl*ref_xyh[ni][2]/cur;                            // 初始化分辨率
        double resolution = 2*(ref_xyh[ni][2] - minh)*0.4996716774/480; //视野范围地面最低高度
        
        // 计算光学序列轨迹
        double a,b,c;                                // 初始化光学序列对应相机坐标
        a = ref_xyh[ni][0] + resolution*offset.x;                // x坐标
        b = ref_xyh[ni][1] + resolution*offset.y;                // y坐标
        c = ref_xyh[ni][2] + dist(generator);                    // z坐标（带高斯白噪声）
        Optical_xyh.push_back(Vec3(a, b, c));                    // 储存
        
        double qw, qx, qy, qz;
        qw = ref_qwxyz[ni][0];
        qx = ref_qwxyz[ni][1];
        qy = ref_qwxyz[ni][2];
        qz = ref_qwxyz[ni][3];
        // 输出流：序号，x偏移像素值，y偏移像素值，x坐标，y坐标，z坐标
        f << float(ni+1)/10 <<" "<< a <<" "<< b <<" "<< c <<" "<< qw <<" "<< qx <<" "<< qy <<" "<< qz <<endl;
//         f << float(ni+1)/10 <<" "<< offset.x <<" "<< offset.y <<" "<< response << " "<< resolution << endl;
        
    }
    
    // 保存至txt文件
    f.close();
    
    std::cout << endl << "Translation matcher outcome saved!" << std::endl;
    return 0;
}

void LoadImagesMars(const string &sImgDir, 
                    vector<string> &vstrImageFilenames) 
{
    std::vector<std::string> vecFileNames;

    if (!stlplus::folder_exists(sImgDir)) 
    {
        std::cerr << "Invalid input image directory" << std::endl;
    } 
    else 
    {
        vecFileNames = stlplus::folder_wildcard(sImgDir, "*.jpg", false, true);
        std::sort(vecFileNames.begin(), vecFileNames.end());
    }

    std::cout << "Number of image: " << vecFileNames.size() << std::endl;

    for (size_t i = 0; i < vecFileNames.size(); ++i) 
    {
        std::string sImgName = vecFileNames[i];
        std::string sPath = stlplus::create_filespec(sImgDir, sImgName);
        vstrImageFilenames.push_back(sPath);
    }
}

void LoadLocation(const string &strPathToSequence, 
                  vector<Vec3> &vstrtrajectoryFilenames, 
                  vector<Vec4> &vstrquaterniondFilenames)
{

    // 读Height.txt中的数据
    ifstream f;
    string strPathXYFile = strPathToSequence;

    f.open(strPathXYFile.c_str());
    while (!f.eof()) 
    {
        string s;
        getline(f, s);
        if (!s.empty()) 
        {
            stringstream ss;
            ss << s;
            double t;
            Vec3 ref_xyh;
            Vec4 ref_qwxyz;
            ss >> t >> ref_xyh(0) >> ref_xyh(1)>>ref_xyh(2)>>ref_qwxyz(0)>>ref_qwxyz(1)>>ref_qwxyz(2)>>ref_qwxyz(3);
            
            vstrtrajectoryFilenames.push_back(ref_xyh);
            vstrquaterniondFilenames.push_back(ref_qwxyz);
        }
    }                       
}

//     cv::Size dst_sz = im2.size();
//     cv::Mat t_mat =cv::Mat::zeros(2, 3, CV_32FC1);
//     t_mat.at<float>(0, 0) = 1;
// 	t_mat.at<float>(0, 2) = 100;
// 	t_mat.at<float>(1, 1) = 1;
// 	t_mat.at<float>(1, 2) = 50; 
// 	cv::warpAffine(im2, im2, t_mat, dst_sz);
