#pragma once
#include <string>

class SignalInput
{

public:

	SignalInput();
	~SignalInput();

	void set_mode(int);
	void set_index(int);
	void set_name(std::string);
	void add_name(char);
	int get_mode();
	int get_index();
	std::string get_name();

private:
	int mode;
	int index;
	std::string name;
};

