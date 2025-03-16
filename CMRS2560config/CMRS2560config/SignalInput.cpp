#include "SignalInput.h"

SignalInput::SignalInput()
{
	mode = 0;
	index = 0;
	name = "";
}

SignalInput::~SignalInput()
{

}

void SignalInput::set_mode(int arg)
{
	mode = arg;
}

void SignalInput::set_index(int arg)
{
	index = arg;
}

void SignalInput::set_name(std::string arg)
{
	name = arg;
}

int SignalInput::get_mode()
{
	return mode;
}

int SignalInput::get_index()
{
	return index;
}

std::string SignalInput::get_name()
{
	return name;
}

void SignalInput::add_name(char arg)
{
	name.push_back(arg);
}

void SignalInput::clear() {
	mode = 0;
	index = 0;
	name = "";
}