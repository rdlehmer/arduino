#include "QuadSensor.h"

QuadSensor::QuadSensor()
{
	board = 0;
	sensor = 0;
	sensor_num = 0;
	name = "";
}

QuadSensor::~QuadSensor()
{

}

void QuadSensor::set_board(int arg)
{
	board = arg;
}

void QuadSensor::set_sensor(int arg)
{
	sensor = arg;
}

void QuadSensor::set_sensornum(int arg)
{
	sensor_num = arg;
}

void QuadSensor::set_name(std::string arg)
{
	name = arg;
}

int QuadSensor::get_board()
{
	return board;
}

int QuadSensor::get_sensor()
{
	return sensor;
}

int QuadSensor::get_sensornum()
{
	return sensor_num;
}

std::string QuadSensor::get_name()
{
	return name;
}

void QuadSensor::add_name(char arg)
{
	name.push_back(arg);
}