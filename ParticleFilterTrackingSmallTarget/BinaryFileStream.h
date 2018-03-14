#pragma once
#include <fstream>
#include <opencv2/core/core.hpp>

using std::string;
using std::fstream;
using cv::Mat;

class BinaryFileReader
{
public:
	explicit BinaryFileReader(int frameWidth = 0, int frameHeight = 0)
		: _curFrameIndex(0),
		  _frameCount(0),
		  _isAlreadyGetFrameCount(false),
		  _width(frameWidth),
		  _height(frameHeight),
		  _imageSize(frameHeight * frameWidth),
		  _imageDataSize(_imageSize * _pixelSize)
	{
	}

	void Init(string& fileFullName);

	void GetOneFrame(cv::Mat& frame);

private:
	void SetFileFullName(string& fileFullName);

	void InitFileReader();

	void GetFrameCount();

private:
	std::string _fileFullName;
	std::fstream _fin;
	unsigned int _curFrameIndex;
	unsigned int _frameCount;
	bool _isAlreadyGetFrameCount;

	unsigned short _width;
	unsigned short _height;

	unsigned int _imageSize;
	unsigned int _imageDataSize;


	const static unsigned char _pixelSize = 2;
};
