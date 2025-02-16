#pragma once

#include <vector>

typedef std::vector<int, std::allocator<int>> intvect;

class SignalLogic
{
public:
	SignalLogic();
	~SignalLogic();

	void set_signal(int);
	void set_aspect(int);
	void set_firstNOR(int);
	void set_numNOR(int);

	void add_NOR(int,int);
	void clear_NOR();

	int get_signal();
	int get_aspect();
	int get_firstNOR();
	int get_numNOR();
	int get_NOR(int);


private:
	int signal;
	int aspect;
	int firstNOR;
	int numNOR;
	intvect NOR;
};

