#include "BinaryFileStream.h"

void BinaryFileReader::SetFileFullName(std::string& fileFullName)
{
	this->_fileFullName = fileFullName;
}

void BinaryFileReader::SetFrameSize(unsigned short width, unsigned short height)
{
	this->_height = height;
	this->_width = width;
	this->_imageSize = this->_height * this->_width;
	this->_imageDataSize = this->_imageSize * this->_pixelSize;
}

void BinaryFileReader::InitFileReader()
{
	this->_fin.open(this->_fileFullName, std::ios::binary|std::ios::in);
}

void BinaryFileReader::GetOneFrame(cv::Mat& frame)
{
	if(this->_fin.is_open())
	{
		auto frameData = new unsigned char[this->_imageDataSize];
		this->_fin.read(reinterpret_cast<char*>(frameData), this->_imageDataSize);

		memcpy(frame.data, frameData, this->_imageDataSize);

		delete[] frameData;
	}
}
