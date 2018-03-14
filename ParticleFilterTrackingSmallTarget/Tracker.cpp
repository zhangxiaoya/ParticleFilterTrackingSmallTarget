#include "Tracker.h"
#include <iostream>

void Tracker::GetOneFrame(std::string& fileFullName)
{
	if(fileFullName.length() == 0)
	{
		std::cout << "File Name Cannot be Empty!" << std::endl;
		return;
	}

}
