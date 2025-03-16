#pragma once
#include <string>

class QuadSensor
{

public:

	QuadSensor();
	~QuadSensor();

	void set_board(int);
	void set_sensor(int);
	void set_sensornum(int);
	void set_name(std::string);
	void add_name(char);
	int get_board();
	int get_sensor();
	int get_sensornum();
	std::string get_name();
	void clear();

private:
	int board;
	int sensor;
	std::string name;
	int sensor_num;
};

