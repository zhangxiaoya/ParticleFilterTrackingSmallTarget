#include <iostream>
#include "Tracker.h"

#include "BinaryFileStream.h"

using namespace std;

int main()
{
	string fileName = "ir_data_file_20171211_155950975.dat";
	string pathName = "D:\\Bags\\Data\\IRData\\";
	Tracker tracker;

	BinaryFileReader fileReader;
	fileReader.SetFileFullName(pathName + fileName);
	fileReader.SetFrameSize(640, 512);
	fileReader.InitFileReader();

	auto frame = fileReader.GetOneFrame();

	if(frame.empty())
	{
		cout << "Get One Frame Failed!" << endl;
		system("Pause");
		return 0;
	}
	Mat showFrame(640, 512, CV_8UC1);
	for(auto r = 0; r < 512; ++r)
	{
		cv::Ptr<unsigned short> ptrOriginalFrame = frame.ptr<unsigned short>(r);
		cv::Ptr<uchar> ptrDstFrame = frame.ptr<uchar>(r);

		for(auto c = 0; c < 640; ++c)
		{
			unsigned short pixelValue = ptrOriginalFrame[c];
			unsigned lowPixelValue = static_cast<uchar>(pixelValue & 0x00ff);
			ptrDstFrame[c] = lowPixelValue;
		}
	}

	system("Pause");
	return 0;
}