#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

#include "Tracker.h"
#include "BinaryFileStream.h"

#ifndef FrameWidth
#define FrameWidth 640
#endif

#ifndef FrameHeight
#define FrameHeight 512
#endif

int main()
{
    // 定义图像宽和高
    int width = FrameWidth;
    int height = FrameHeight;

    // 文件名格式定义
    string fileFullNameFormat = "/home/ynzhang/Desktop/Data/trackingData/Segment_%02d.dat";
    // 文件名字符串存储
    char fileFullNameArr[200];
    // 初始化文件名
    sprintf(fileFullNameArr, fileFullNameFormat.c_str(), 0);
    // 定义一个文件流读取对象
    BinaryFileReader fileReader(width, height);
    // 初始化文件流读取对象
    fileReader.Init(string(fileFullNameArr));

    // 定义图像帧
    Mat frame(height, width, CV_16UC1);
    // 定义可显示图像帧
    Mat showFrame(height, width, CV_8UC1);

    // 初始化初始位置
    cv::Rect rect(304, 259, 4, 4);
    Tracker tracker(width, height);

    // 设置粒子数量
    tracker.SetParticleCount(100);

    // 是否是第一帧的标识
    bool isFirstFrame = true;
    // 粒子最大权重返回值
    float maxWeight = 0;

    // 目标初始化位置
    int centerX = 306;
    int centerY = 261;

    // 目标初始化宽和高
    int halfWidthOfTarget = 5;
    int halfHeightOfTarget = 5;

    // 从哪一个文件开始，目标进入视野
    int startFileIndex = 9;
    // 到哪一个文件结束，目标离开视野
    int endFileIndex = 10;

    // 循环遍历所有的图像文件
    for(auto fileIdx = startFileIndex; fileIdx < endFileIndex; ++ fileIdx)
    {
        // 格式化文件名
        sprintf(fileFullNameArr, fileFullNameFormat.c_str(), fileIdx);
        // 读取文件流对象重新初始化读取操作
        fileReader.ResetFileStream(string(fileFullNameArr));
        // 打印文件编号
        std::cout << "File Index = " << std::setw(6) << fileIdx << std::endl;

        // 遍历该文件中的图像
        auto frameIndex = 0;

        unsigned short* imgDataPointer = nullptr;
        while (fileReader.GetOneFrame(frame, imgDataPointer))
        {
            if(isFirstFrame)
            {
                tracker.Initialize(centerX,
                                   centerY,
                                   halfWidthOfTarget,
                                   halfHeightOfTarget,
                                   imgDataPointer,
                                   width,
                                   height);

                isFirstFrame = false;
            }
            frameIndex++;

            auto trackingStatus = tracker.ParticleTracking(imgDataPointer,
                                                           width, height,
                                                           centerX, centerY,
                                                           halfWidthOfTarget,
                                                           halfHeightOfTarget,
                                                           maxWeight);


            std::cout << "Max weight = " << std::setw(10) << maxWeight << std::endl;

            // 可视化低8位像素信息
            for (auto r = 0; r < FrameHeight; ++r)
            {
                auto ptrOriginal = showFrame.ptr<uchar>(r);
                auto ptrResult = frame.ptr<unsigned short>(r);
                for (auto c = 0; c < FrameWidth; ++c)
                {
                    auto pixelValue = ptrResult[c];
                    ptrOriginal[c] = static_cast<unsigned char>(pixelValue & 0x00ff);
                }
            }

            if(trackingStatus == 1 || maxWeight > 0.0001)
            {
                cv::rectangle(showFrame,
                              cv::Point(centerX - halfWidthOfTarget, centerY - halfHeightOfTarget),
                              cv::Point(centerX + halfWidthOfTarget, centerY + halfHeightOfTarget),
                              cv::Scalar(255, 0, 0), 1, 8, 0);
            }
            else
            {
                std::cout << "Target Lost" << std::endl;
            }

            imshow("Frame", showFrame);
            cv::waitKey(100);
        }
        std::cout << "All frame count is " << frameIndex << std::endl;
        cv::waitKey(1);
    }

    // 销毁显示图像窗口句柄
    cv::destroyAllWindows();

    return 0;
}