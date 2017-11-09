#pragma once
#include <string>
#include <iostream>

//#include "fms.h"

namespace vultron
{
	class error
	{
	private:
		std::string _message;
		std::string _classname;
		int _line;
	public:
		error(std::string message, std::string classname, int line) :_message(message), _classname(classname), _line(line){}
		std::string toString()
		{
			return "Message: " + _message + " In class " + _classname + ".";
		}
	};
}