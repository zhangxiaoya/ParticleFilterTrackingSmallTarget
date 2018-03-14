#pragma once
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>

using std::string;
using std::fstream;
using cv::Mat;

class BinaryFileReader
{
public:
	BinaryFileReader()
		: _curFrameIndex(0),
		  _frameCount(0), _width(0), _height(0), _imageSize(0), _imageDataSize(0)
	{
	}

	void SetFileFullName(string& fileFullName);

	void SetFrameSize(unsigned width, unsigned height);

	void InitFileReader();

	Mat GetOneFrame();

private:
	std::string _fileFullName;
	std::fstream _fin;
	unsigned int _curFrameIndex;
	unsigned int _frameCount;

	unsigned char _width;
	unsigned char _height;

	unsigned short _imageSize;
	unsigned int _imageDataSize;

	const static unsigned char _pixelSize = 2;
};
