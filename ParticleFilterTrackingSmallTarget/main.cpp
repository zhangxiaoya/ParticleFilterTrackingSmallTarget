#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "BinaryFileStream.h"
#include <iomanip>

using namespace std;

#define FrameWidth 640
#define FrameHeight 512

int main()
{
	string fileFullNameFormat = "D:\\Bags\\Data\\IRData\\trackingData\\Segment_%02d.dat";
	string fileFullName = "D:\\Bags\\Data\\IRData\\trackingData\\Segment_00.dat";

	char fileFullNameArr[200];

	BinaryFileReader fileReader(FrameWidth, FrameHeight);
	fileReader.Init(fileFullName);

	Mat frame(FrameHeight, FrameWidth, CV_16UC1);
	Mat showFrame(FrameHeight, FrameWidth, CV_8UC1);

	for(auto fileIdx = 0; fileIdx < 42; ++ fileIdx)
	{

		sprintf(fileFullNameArr, fileFullNameFormat.c_str(), fileIdx);

		fileReader.ResetFileStream(string(fileFullNameArr));

		cout << "File Index = " << setw(6) << fileIdx << endl;

		int frameIndex = 0;
		while (fileReader.GetOneFrame(frame))
		{
//			cout << "Current Frame Index is " << setw(4) << frameIndex++ << endl;

			frameIndex++;
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

			imshow("Frame", showFrame);
			cv::waitKey(1);
		}
		cout << "All frame count is " << frameIndex << endl;
		cv::waitKey(1);
	}

	cv::destroyAllWindows();

	return 0;
}