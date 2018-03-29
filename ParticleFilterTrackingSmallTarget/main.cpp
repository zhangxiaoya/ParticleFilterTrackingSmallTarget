	#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "BinaryFileStream.h"
#include <iomanip>

#define FrameWidth 640
#define FrameHeight 512

int main()
{
	// �ļ�����ʽ����
	string fileFullNameFormat = "D:\\Bags\\Data\\IRData\\trackingData\\Segment_%02d.dat";
	// �ļ����ַ����洢
	char fileFullNameArr[200];

	// ��ʼ������
	// ��ʼ���ļ���
	sprintf_s(fileFullNameArr, fileFullNameFormat.c_str(), 0);
	// ��ʼ���ļ�����ȡ����
	BinaryFileReader fileReader(FrameWidth, FrameHeight);
	fileReader.Init(string(fileFullNameArr));

	// ����ͼ��֡
	Mat frame(FrameHeight, FrameWidth, CV_16UC1);
	// �������ʾͼ��֡
	Mat showFrame(FrameHeight, FrameWidth, CV_8UC1);

	// ��ʼ����ʼλ��
	cv::Rect rect(304, 259, 4, 4);
	Tracker tracker(FrameWidth,FrameHeight);

	bool isFirstFrame = true;
	float maxWeight = 0;
	int centerX = 306;
	int centerY = 261;
	int width = FrameWidth;
	int height = FrameHeight;
	int halfWidthOfTarget = 5;
	int halfHeightOfTarget = 5;

	// ѭ���������е�ͼ���ļ�
	for(auto fileIdx = 9; fileIdx < 10; ++ fileIdx)
	{

		sprintf(fileFullNameArr, fileFullNameFormat.c_str(), fileIdx);

		fileReader.ResetFileStream(string(fileFullNameArr));
		// ��ӡ�ļ����
		std::cout << "File Index = " << std::setw(6) << fileIdx << std::endl;

		// ѭ���������ļ��е�����ͼ��
		auto frameIndex = 0;

		unsigned short* imgDataPointer = nullptr;
		while (fileReader.GetOneFrame(frame, imgDataPointer))
		{
			if(isFirstFrame)
			{
				tracker.Initialize(306, 261, halfWidthOfTarget, halfHeightOfTarget, imgDataPointer, FrameWidth, FrameHeight);
				isFirstFrame = false;
			}
			frameIndex++;

			auto trackingStatus = tracker.ParticleTracking(imgDataPointer, FrameWidth, FrameHeight, centerX, centerY, halfWidthOfTarget, halfHeightOfTarget, maxWeight);


			// ���ӻ���8λ������Ϣ
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

	// ������ʾͼ�񴰿ھ��
	cv::destroyAllWindows();

	return 0;
}