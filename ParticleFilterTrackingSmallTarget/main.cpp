#include <iostream>
#include <highgui/highgui.hpp>

#include "Tracker.h"
#include "BinaryFileStream.h"
#include <iomanip>

#define FrameWidth 640
#define FrameHeight 512

int main()
{
	// 文件名格式定义
	string fileFullNameFormat = "D:\\Bags\\Data\\IRData\\trackingData\\Segment_%02d.dat";
	// 文件名字符串存储
	char fileFullNameArr[200];

	// 初始化操作
	// 初始化文件名
	sprintf_s(fileFullNameArr, fileFullNameFormat.c_str(), 0);
	// 初始化文件流读取对象
	BinaryFileReader fileReader(FrameWidth, FrameHeight);
	fileReader.Init(string(fileFullNameArr));

	// 定义图像帧
	Mat frame(FrameHeight, FrameWidth, CV_16UC1);
	// 定义可显示图像帧
	Mat showFrame(FrameHeight, FrameWidth, CV_8UC1);

	// 初始化初始位置
	cv::Rect rect(304, 259, 4, 4);
	Tracker tracker(FrameWidth,FrameHeight);

	bool isFirstFrame = true;
	float maxWeight = 0;
	int centerX = 306;
	int centerY = 261;
	int width = FrameWidth;
	int height = FrameHeight;
	int halfWidthOfTarget = 2;
	int halfHeightOfTarget = 2;

	// 循环遍历所有的图像文件
	for(auto fileIdx = 9; fileIdx < 10; ++ fileIdx)
	{
		// 格式化文件名
		sprintf_s(fileFullNameArr, fileFullNameFormat.c_str(), fileIdx);
		// 读取文件流对象重新初始化读取操作
		fileReader.ResetFileStream(string(fileFullNameArr));
		// 打印文件编号
		std::cout << "File Index = " << std::setw(6) << fileIdx << std::endl;

		// 循环遍历该文件中的所有图像
		auto frameIndex = 0;

		unsigned short* imgDataPointer = nullptr;
		while (fileReader.GetOneFrame(frame, imgDataPointer))
		{
			if(isFirstFrame)
			{
				tracker.Initialize(306, 261, 2, 2, imgDataPointer, FrameWidth, FrameHeight);
				isFirstFrame = false;
			}
			frameIndex++;

			auto trackingStatus = tracker.ParticleTracking(imgDataPointer, FrameWidth, FrameHeight, centerX, centerY, halfWidthOfTarget, halfHeightOfTarget, maxWeight);


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
			cv::waitKey(1000);
		}
		std::cout << "All frame count is " << frameIndex << std::endl;
		cv::waitKey(1);
	}

	// 销毁显示图像窗口句柄
	cv::destroyAllWindows();

	system("pause");
	return 0;
}