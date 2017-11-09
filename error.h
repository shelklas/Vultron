/*
* Name: Sheldon Klassen
* Class: error.h
* Description:
*/

#pragma once
#include <string>
#include <iostream>

namespace vultron
{
	class error
	{
	private:
		std::string _message;
		std::string _function;
		int _line;
	public:
		error(std::string message, std::string function, int line) :_message(message), _function(function), _line(line){}
		std::string toString()
		{
			return "MESSAGE: " + _message + "\nFUNCTION: " + _function + ".\nLINE: " + std::to_string(_line);
		}
	};
}