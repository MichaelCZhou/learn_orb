/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    //命令行参数
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    //vector一种数据结构，确切的说是一个类，相当于一个动态数组，一定加上using namespce std;
    // 第一次载入图像,由时间戳得到图片数量,并依序存入容器中.
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    //size()指目前存在的元素个数.由上面得知.
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //创建一个SLAM系统。
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    //vTimesTrack为跟踪时间的统计数据
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    //resize()重新指定有效元素的个数，区别于reserve()指定容器能存储数据的个数。另外，capacity()指容器能存储数据的个数

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop主循环
    cv::Mat imLeft, imRight;
    for(int ni = 0; ni < nImages; ni++)
    {
        // Read left and right images from file
        //(CV_LOAD_IMAGE_ANYCOLOR和CV_LOAD_IMAGE_UNCHANGED是等值的）8bit,
        //表示不对图像进行任何处理，按照原始图像类型将图像从load函数中的容器中载入
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

        //第ni帧的时间戳
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            //插入器<<向流输出数据,析取器>>从流中输入数据
            //cout：写到标准输出的ostream对象；
            //cerr：不经过缓冲直接输出到标准错误的ostream对象，常用于程序错误信息；
                 //目的就是在你最需要它的紧急情况下，还能得到输出功能的支持
            //clog：也是输出标准错误流（这点儿和cerr是一样的），貌似平时很少用到这个啊；
            return 1;
        }

        //计时开始
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        //SLAM访问TrackStereo（入口）处理核心
        SLAM.TrackStereo(imLeft,imRight,tframe);        

        //跟踪完一对图像后计时结束
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        //ttrack为两帧图像之间的时间差
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        //将时间差存入记录时间数据的容器
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        //等待下一帧(未看:
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
			this_thread::sleep_for(std::chrono::microseconds((int)((T-ttrack)*1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    //100ms = 10Hz,66.7ms = 15Hz,50ms = 20Hz,33.33ms = 30Hz,20ms = 50Hz,10ms = 100Hz,
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");//???QObject::~QObject: Timers cannot be stopped from another thread

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    //open("file.txt");
    //从一个string得到c类型的字符数组：c_str()、data()、copy(p,n)。
    //返回当前字符串的首字符地址。换种说法，c_str()函数返回一个指向正规C字符串的指针常量，内容与本string串相同。
    //c_str()的原型是：const char*c_str() const;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    //用const表示不会修改这个参数.
    //由时间戳多少决定分配给图像空间的大小
    const int nTimes = vTimestamps.size();   
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        //std::setw ：需要填充多少个字符,默认填充的字符为' '空格
        //std::setfill：设置std::setw将填充什么样的字符，如:std::setfill('*')
        //std::setbase(n)：将输出数据转换为n进制
        //std::setprecision()：控制输出流显示浮点数的数字个数，C++默认的流输出数值有效位是6。
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
