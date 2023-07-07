/******************************************************************************
* Copyright (C) 2019 Italian Institute of Technology
*
*    Developers:
*    Fernando Caponetto (2019-, fernando.caponetto@iit.it)
*
******************************************************************************/

#ifndef _DEVICE_EXCEPION_H_
#define _DEVICE_EXCEPION_H_
#include <iostream>

namespace myDevice
{

	class DeviceException : public std::runtime_error
	{
		// std::string what_message;
	public:

		DeviceException(const std::string & message) : runtime_error(message) {};

		// virtual const char* what() const throw ()
		// {
		// 	return what_message.c_str();
		// }
	};

}
#endif
