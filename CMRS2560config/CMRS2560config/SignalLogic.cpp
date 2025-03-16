#include "SignalLogic.h"

SignalLogic::SignalLogic()
{
	signal = 0;
	aspect = 0;
	firstNOR = 0;
	numNOR = 0;
}

SignalLogic::~SignalLogic()
{

}

void SignalLogic::set_signal(int arg)
{
	signal = arg;
}

void SignalLogic::set_aspect(int arg)
{
	aspect = arg;
}

void SignalLogic::set_firstNOR(int arg)
{
	firstNOR = arg;
}

void SignalLogic::set_numNOR(int arg)
{
	numNOR = arg;
}

int SignalLogic::get_signal()
{
	return signal;
}

int SignalLogic::get_aspect()
{
	return aspect;
}

int SignalLogic::get_firstNOR()
{
	return firstNOR;
}

int SignalLogic::get_numNOR()
{
	return numNOR;
}

void SignalLogic::add_NOR(int index, int arg)
{
	NOR.push_back(arg);
}

int SignalLogic::get_NOR(int index)
{
	int ret_val = NOR[index];
	return ret_val;
}

void SignalLogic::clear_NOR() {
	NOR.clear();
}

void SignalLogic::clear() {
	signal = 0;
	aspect = 0;
	firstNOR = 0;
	numNOR = 0;
}