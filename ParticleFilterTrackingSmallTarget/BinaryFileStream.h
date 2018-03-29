#pragma once
#include <fstream>
#include <opencv2/core/core.hpp>

using std::string;
using std::fstream;
using cv::Mat;

class BinaryFileReader
{
public:
	explicit BinaryFileReader(int frameWidth, int frameHeight)
			: _curFrameIndex(0),
			  _frameCount(0),
			  _isAlreadyGetFrameCount(false),
			  _width(frameWidth),
			  _height(frameHeight),
			  _imageSize(frameHeight * frameWidth),
			  _imageDataSize(_imageSize * _pixelSize),
			  _imgData(nullptr)
	{
	}

	~BinaryFileReader();

	void ReleaseReader();

	void ResetFileStream(string fileFullName);

	void Init(string fileFullName);

	bool GetOneFrame(cv::Mat& frame, unsigned short*& imgData);

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

	unsigned short* _imgData;

	const static unsigned char _pixelSize = 2;
};
