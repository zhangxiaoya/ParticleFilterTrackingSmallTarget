#include "BinaryFileStream.h"
#include <iostream>

BinaryFileReader::~BinaryFileReader()
{
	this->ReleaseReader();
}

void BinaryFileReader::ReleaseReader()
{
	this->_fin.close();
}

void BinaryFileReader::ResetFileStream(string fileFullName)
{
	SetFileFullName(fileFullName);
	InitFileReader();
	this->_curFrameIndex = 0;
}

void BinaryFileReader::Init(string& fileFullName)
{
	SetFileFullName(fileFullName);

	InitFileReader();

	GetFrameCount();
}

void BinaryFileReader::SetFileFullName(std::string& fileFullName)
{
	this->_fileFullName = fileFullName;
}

void BinaryFileReader::InitFileReader()
{
	if (this->_fin.is_open())
	{
		this->_fin.close();
	}
	try
	{
		this->_fin.open(this->_fileFullName, std::ios::binary | std::ios::in);
	}
	catch (std::ios::failure& e)
	{
		std::cout << "Exception opening / reading / closing file\n";
	}
}

bool BinaryFileReader::GetOneFrame(cv::Mat& frame)
{
	if(_curFrameIndex >= _frameCount)
		return false;
	if(this->_fin.is_open())
	{
		auto frameData = new unsigned char[this->_imageDataSize];
		this->_fin.read(reinterpret_cast<char*>(frameData), this->_imageDataSize);

		memcpy(frame.data, frameData, this->_imageDataSize);

		delete[] frameData;
		_curFrameIndex++;
		return true;
	}
	return false;
}

void BinaryFileReader::GetFrameCount()
{
	if(this->_isAlreadyGetFrameCount == false)
	{
		if (this->_fin.is_open())
		{
			auto curPos = this->_fin.tellg();
			this->_fin.seekg(0, std::ios::beg);
			auto begPos = this->_fin.tellg();
			this->_fin.seekg(0, std::ios::end);
			auto len = this->_fin.tellg() - begPos;
			this->_fin.seekg(0, std::ios::beg);

			this->_frameCount = len / this->_imageDataSize;
			this->_isAlreadyGetFrameCount = true;
		}
		else
		{
			this->_isAlreadyGetFrameCount = false;
			this->_frameCount = 0;
		}
	}
}
