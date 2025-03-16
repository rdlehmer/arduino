#pragma once
class Indicator
{
public:
	Indicator();
	~Indicator();

	void set_switch(int);
	void set_sensor(int);

	int get_switch();
	int get_sensor();

	void clear();

private:
	int sw_number;
	int sensor_num;
};

