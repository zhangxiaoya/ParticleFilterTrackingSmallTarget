#pragma once
#include <string>

class Tracker
{
public:
	explicit Tracker(unsigned char width = 0, unsigned char height = 0)
		: _width(width),
		  _height(height),
	_curFrame(nullptr)
	{
	}

	void GetOneFrame(std::string& fileFullName);

private:
	unsigned char _width;  // Frame size : width
	unsigned char _height; // Frame size : height

	unsigned short* _curFrame;
};
