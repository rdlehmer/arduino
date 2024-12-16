#include "Turnout.h"
#include <iostream>

Turnout::Turnout()
{
	toggle = 0;
	name = "\0";
}

Turnout::~Turnout()
{

}

void Turnout::set_toggle(int arg)
{
	toggle = arg;
}

void Turnout::set_name(std::string arg)
{
	name = arg;
}

int Turnout::get_toggle()
{
	return toggle;
}

std::string Turnout::get_name()
{
	return name;
}

void Turnout::add_name(char arg)
{
//	std::cout << arg;
	name.push_back(arg);
}