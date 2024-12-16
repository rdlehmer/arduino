#include "Indicator.h"

Indicator::Indicator()
{
	sw_number = 0;
	sensor_num = 0;
}

Indicator::~Indicator()
{

}

void Indicator::set_switch(int arg)
{
	sw_number = arg;
}

void Indicator::set_sensor(int arg)
{
	sensor_num = arg;
}

int Indicator::get_switch()
{
	return sw_number;
}

int Indicator::get_sensor()
{
	return sensor_num;
}