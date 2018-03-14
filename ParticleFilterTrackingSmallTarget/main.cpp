#include <iostream>
#include "Tracker.h"

#include "BinaryFileStream.h"

using namespace std;

#define FrameWidth 640
#define FrameHeight 512

int main()
{
	string fileName = "ir_data_file_20171211_155950975.dat";
	string pathName = "D:\\Bags\\Data\\IRData\\";
	Tracker tracker;

	BinaryFileReader fileReader;
	fileReader.SetFileFullName(pathName + fileName);
	fileReader.SetFrameSize(640, 512);
	fileReader.InitFileReader();

	Mat frame(FrameHeight, FrameWidth, CV_16UC1);
	fileReader.GetOneFrame(frame);

	if(frame.empty())
	{
		cout << "Get One Frame Failed!" << endl;
		system("Pause");
		return 0;
	}
	Mat showFrame(FrameHeight, FrameWidth, CV_8UC1);

	for(auto r = 0; r < FrameHeight; ++r)
	{
		auto ptr = showFrame.ptr<uchar>(r);
		auto ptr1 = frame.ptr<unsigned short>(r);
		for(auto c = 0; c < FrameWidth; ++c)
		{
			unsigned short pixelValue = ptr1[c];
			ptr[c] = static_cast<unsigned char>(pixelValue & 0x00ff);
		}
	}

	system("Pause");
	return 0;
}