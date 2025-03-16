#pragma once

#include <string>

class Signal
{
public:
	Signal();
	~Signal();

	void set_active(int);
	void set_signalhead(std::string);
	void set_leadingsignal(std::string);

	void add_signalhead(char);
	void add_leadingsignal(char);

	int get_active();
	std::string get_signalhead();
	std::string get_leadingsignal();

	void clear();

private:
	int active;
	std::string signalhead;
	std::string leadingsignal;
};

