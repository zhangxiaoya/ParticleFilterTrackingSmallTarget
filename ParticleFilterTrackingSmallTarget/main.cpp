#include <iostream>
#include <highgui/highgui.hpp>

#include "Tracker.h"
#include "BinaryFileStream.h"
#include <iomanip>

using namespace std;

#define FrameWidth 640
#define FrameHeight 512

int main()
{
	string fileFullName = "D:\\Bags\\Data\\IRData\\ir_data_file_20171211_155950975.dat";
	Tracker tracker;

	BinaryFileReader fileReader(FrameWidth, FrameHeight);
	fileReader.Init(fileFullName);

	Mat frame(FrameHeight, FrameWidth, CV_16UC1);
	Mat showFrame(FrameHeight, FrameWidth, CV_8UC1);

	int frameIndex = 0;
	while(fileReader.GetOneFrame(frame))
	{
		cout << "Current Frame Index is " << setw(4) << frameIndex++ << endl;

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
		cv::waitKey(10);
	}
	cout << "All frame count is " << frameIndex << endl;

	cv::destroyAllWindows();

	system("pause");
	return 0;
}