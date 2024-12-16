#pragma once

#include <string>

class Turnout
{

public:

	Turnout();
	~Turnout();

	void set_toggle(int);
	void set_name(std::string);
	void add_name(char);
	int get_toggle();
	std::string get_name();

private:
	int toggle;
	std::string name;
};

