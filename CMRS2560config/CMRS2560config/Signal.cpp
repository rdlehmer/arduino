#include "Signal.h"

Signal::Signal()
{
	active = 0;
	signalhead = "";
	leadingsignal = "";
}

Signal::~Signal()
{

}

void Signal::set_active(int arg)
{
	active = arg;
}

int Signal::get_active()
{
	return active;
}

void Signal::set_signalhead(std::string arg)
{
	signalhead = arg;
}

void Signal::set_leadingsignal(std::string arg)
{
	leadingsignal = arg;
}

std::string Signal::get_signalhead()
{
	return signalhead;
}

std::string Signal::get_leadingsignal()
{
	return leadingsignal;
}

void Signal::add_signalhead(char arg)
{
	signalhead.push_back(arg);
}

void Signal::add_leadingsignal(char arg)
{
	leadingsignal.push_back(arg);
}